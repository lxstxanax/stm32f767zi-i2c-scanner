#ifndef MAX17320_H
#define MAX17320_H

/*
 * MAX17320 I2C driver: presence check, word read/write, backup, shadow-RAM
 * config write + readback verify, remaining-NVM-writes query, and (gated)
 * NVM commit.
 *
 * All addresses/commands/timings below are taken from the MAX17320 datasheet
 * (Documents/xnx_datasheets/max17320.pdf and Documents/Altium/datasheets/
 * max17320.pdf), not guessed:
 *   - Table 116 "2-Wire Slave Addresses": slave 6Ch (8-bit) / 0x36 (7-bit)
 *     addresses memory 000h-0FFh 1:1; slave 16h (8-bit) / 0x0B (7-bit)
 *     addresses memory 100h-1FFh via address byte = (memory - 0x100), i.e.
 *     for the NV block 180h-1EFh the on-wire byte is 80h-EFh.
 *   - "Nonvolatile Memory Commands" / "Nonvolatile Block Programming" /
 *     "Determining Number of Remaining Updates" sections (datasheet pages
 *     ~127-130) for CommStat/Command register sequences, command codes and
 *     the remaining-updates decode algorithm.
 *   - Electrical Characteristics table for tBLOCK/tRECALL timing.
 */

#include <stdint.h>
#include <stddef.h>
#include "stm32f7xx_hal.h"
#include "max17320_config.h"

/* ---- I2C addresses: 7-bit vs HAL's 8-bit (shifted, R/W-bit-in-bit0) ----
 * HAL_I2C_Master_* / HAL_I2C_IsDeviceReady all want the 8-bit form
 * (7-bit address already shifted left by 1); keep the two representations
 * distinct so callers never have to remember to shift. */
#define MAX17320_ADDR7_MAIN        (0x36u)  /* datasheet Table 116: slave 6Ch */
#define MAX17320_ADDR7_NV          (0x0Bu)  /* datasheet Table 116: slave 16h */
#define MAX17320_HAL_ADDR_MAIN     ((uint16_t)(MAX17320_ADDR7_MAIN << 1))
#define MAX17320_HAL_ADDR_NV       ((uint16_t)(MAX17320_ADDR7_NV   << 1))

typedef enum {
    MAX17320_OK = 0,
    MAX17320_ERR_NOT_PRESENT,
    MAX17320_ERR_I2C,
    MAX17320_ERR_MISMATCH,
    MAX17320_ERR_NOT_CONFIRMED,
    MAX17320_ERR_NOT_IMPLEMENTED,
    MAX17320_ERR_NVM_ERROR,     /* CommStat.NVError set after Copy NV Block */
} max17320_status_t;

/* Checks both the main and NV I2C addresses respond. */
max17320_status_t max17320_probe(I2C_HandleTypeDef *hi2c);

/* Word (16-bit) register read/write. addr selects which I2C address/byte
 * mapping is used automatically (>= 0x180 -> NV slave, address byte =
 * addr & 0xFF per Table 116; else main slave, address byte = addr). */
max17320_status_t max17320_read_reg(I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t *value);
max17320_status_t max17320_write_reg(I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t value);

/* Reads every register listed in max17320_target_config into out[],
 * out[] must have MAX17320_TARGET_CONFIG_COUNT entries. Pure readback,
 * touches nothing. Use this BEFORE any write to save the current state. */
max17320_status_t max17320_backup_config(I2C_HandleTypeDef *hi2c, uint16_t *out, size_t out_len);

/* Writes max17320_target_config[] to the device's shadow RAM (0x180-0x1EF
 * registers). This does NOT touch nonvolatile memory -- it is lost on
 * power cycle until max17320_commit_nvm() is run. Safe to call repeatedly. */
max17320_status_t max17320_write_shadow_config(I2C_HandleTypeDef *hi2c);

/* Reads back every register in max17320_target_config and compares against
 * the expected value. Returns MAX17320_OK if all match, MAX17320_ERR_MISMATCH
 * otherwise. mismatches/mismatch_cap let the caller collect the differing
 * entries (indices into max17320_target_config[]); pass NULL/0 to skip. */
max17320_status_t max17320_verify_shadow_config(I2C_HandleTypeDef *hi2c,
                                                 size_t *mismatches,
                                                 size_t mismatch_cap,
                                                 size_t *mismatch_count);

/*
 * Determining Number of Remaining Updates (datasheet p.129-130):
 * the config-memory NV block can only be physically written 7 times total
 * (the 1st happens at Maxim's factory test). This queries how many of
 * those 7 have been consumed. ALWAYS call this and show the result to the
 * user before ever calling max17320_commit_nvm().
 */
max17320_status_t max17320_read_remaining_nvm_updates(I2C_HandleTypeDef *hi2c,
                                                       uint8_t *used,
                                                       uint8_t *remaining);

/*
 * Commits the current shadow RAM contents (0x180-0x1EF, excluding the
 * factory-locked 0x1BC-0x1BF nROMIDx) to nonvolatile memory, following the
 * datasheet's 12-step "Nonvolatile Block Programming" sequence exactly
 * (Copy NV Block command 0xE904, tBLOCK wait, NVError check, full reset,
 * POR wait, re-lock write protection).
 *
 * This makes an irreversible dent in the part's limited write budget
 * (7 total, factory test already used 1) EVEN IF THE RESULT IS WRONG.
 * Do not call this speculatively. Before calling:
 *   1. max17320_backup_config()
 *   2. max17320_write_shadow_config() + max17320_verify_shadow_config()
 *      until it reports MAX17320_OK (shadow RAM is volatile/reversible)
 *   3. max17320_read_remaining_nvm_updates() and show it to the user
 *   4. get an explicit, deliberate human confirmation
 *
 * Gated twice on purpose:
 *   - compile-time: only actually runs if MAX17320_I_KNOW_THIS_BURNS_NVM
 *     is defined to 1 in the build (off by default). With it undefined/0,
 *     this always returns MAX17320_ERR_NOT_IMPLEMENTED and touches nothing.
 *   - run-time: confirm_token must equal MAX17320_NVM_CONFIRM_TOKEN, which
 *     the caller must set deliberately (e.g. only after the interactive
 *     confirmation in step 4 above), else MAX17320_ERR_NOT_CONFIRMED.
 */
#define MAX17320_NVM_CONFIRM_TOKEN (0xA5A5C0DEu)
max17320_status_t max17320_commit_nvm(I2C_HandleTypeDef *hi2c, uint32_t confirm_token);

/*
 * Re-runs ONLY the post-write housekeeping (re-unlock, pulse Config2.POR_CMD
 * to reset the fuel-gauge algorithm state, wait for it to clear, re-lock)
 * -- i.e. the tail of max17320_commit_nvm() that does NOT touch nonvolatile
 * memory. Safe to call repeatedly / on its own: if max17320_commit_nvm()
 * already reported NVError clear (data committed) but then failed later at
 * this stage, call this instead of max17320_commit_nvm() again -- it never
 * repeats the Copy NV Block step and so never consumes another lifetime
 * write. No confirm_token needed since it cannot touch NVM.
 */
max17320_status_t max17320_finish_post_commit_reset(I2C_HandleTypeDef *hi2c);

#endif /* MAX17320_H */
