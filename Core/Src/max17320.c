#include "max17320.h"
#include <stdio.h>

/* Datasheet register/command constants (all main-address-space, < 0x180) */
#define REG_COMMAND             0x060u
#define REG_COMMSTAT            0x061u
#define REG_CONFIG2             0x0ABu
#define REG_NV_REMAINING_FLAGS  0x1FDu   /* only valid right after RECALL_REMAINING_UPDATES */

#define CMD_COPY_NV_BLOCK              0xE904u
#define CMD_RECALL_REMAINING_UPDATES   0xE29Bu
#define CMD_FULL_RESET                 0x000Fu
#define CONFIG2_POR_CMD                0x8000u
#define CONFIG2_POR_CMD_BIT            (1u << 15)

#define COMMSTAT_UNLOCK          0x0000u
#define COMMSTAT_LOCK            0x00F9u
#define COMMSTAT_NVERROR_MASK    0x0004u

/* Electrical Characteristics (max values) + small margin */
#define T_BLOCK_MS   7360u
#define T_RECALL_MS  20u
/* Datasheet does not give an explicit max wait for Config2.POR_CMD to
 * self-clear (only tPOR=10ms for power-up POR, which may not be the same
 * timing). Observed on real hardware: didn't clear within 200ms twice in a
 * row even though the preceding Copy NV Block had already succeeded
 * (NVError clear) -- so use a much larger margin here since this stage
 * does not touch NVM and is safe to wait on / repeat. */
#define T_POR_POLL_TIMEOUT_MS 3000u

static uint8_t addr7_for(uint16_t reg_addr)
{
    return (reg_addr >= 0x180u) ? MAX17320_ADDR7_NV : MAX17320_ADDR7_MAIN;
}

max17320_status_t max17320_probe(I2C_HandleTypeDef *hi2c)
{
    if (HAL_I2C_IsDeviceReady(hi2c, MAX17320_HAL_ADDR_MAIN, 2, 50) != HAL_OK) {
        return MAX17320_ERR_NOT_PRESENT;
    }
    if (HAL_I2C_IsDeviceReady(hi2c, MAX17320_HAL_ADDR_NV, 2, 50) != HAL_OK) {
        return MAX17320_ERR_NOT_PRESENT;
    }
    return MAX17320_OK;
}

max17320_status_t max17320_read_reg(I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t *value)
{
    uint16_t hal_addr = (uint16_t)(addr7_for(addr) << 1);
    uint8_t  reg8      = (uint8_t)(addr & 0xFFu); /* Table 116: NV byte = addr - 0x100 == addr & 0xFF */
    uint8_t  rx[2]     = {0};

    if (HAL_I2C_Master_Transmit(hi2c, hal_addr, &reg8, 1, 50) != HAL_OK) {
        return MAX17320_ERR_I2C;
    }
    if (HAL_I2C_Master_Receive(hi2c, hal_addr, rx, 2, 50) != HAL_OK) {
        return MAX17320_ERR_I2C;
    }

    *value = (uint16_t)(rx[0] | ((uint16_t)rx[1] << 8)); /* LSB first on the wire */
    return MAX17320_OK;
}

max17320_status_t max17320_write_reg(I2C_HandleTypeDef *hi2c, uint16_t addr, uint16_t value)
{
    uint16_t hal_addr = (uint16_t)(addr7_for(addr) << 1);
    uint8_t  tx[3];

    tx[0] = (uint8_t)(addr & 0xFFu);
    tx[1] = (uint8_t)(value & 0xFFu);
    tx[2] = (uint8_t)(value >> 8);

    if (HAL_I2C_Master_Transmit(hi2c, hal_addr, tx, 3, 50) != HAL_OK) {
        return MAX17320_ERR_I2C;
    }
    return MAX17320_OK;
}

max17320_status_t max17320_backup_config(I2C_HandleTypeDef *hi2c, uint16_t *out, size_t out_len)
{
    if (out_len < MAX17320_TARGET_CONFIG_COUNT) {
        return MAX17320_ERR_I2C;
    }

    for (size_t i = 0; i < MAX17320_TARGET_CONFIG_COUNT; i++) {
        max17320_status_t st = max17320_read_reg(hi2c, max17320_target_config[i].addr, &out[i]);
        if (st != MAX17320_OK) {
            return st;
        }
    }
    return MAX17320_OK;
}

static max17320_status_t commstat_write(I2C_HandleTypeDef *hi2c, uint16_t value, int times)
{
    for (int i = 0; i < times; i++) {
        max17320_status_t st = max17320_write_reg(hi2c, REG_COMMSTAT, value);
        if (st != MAX17320_OK) {
            return st;
        }
    }
    return MAX17320_OK;
}

max17320_status_t max17320_write_shadow_config(I2C_HandleTypeDef *hi2c)
{
    max17320_status_t st;

    /* Datasheet ("Nonvolatile Block Programming"): shadow RAM writes to the
     * 0x180-0x1EF block are ignored while write protection is set. Clear it
     * first (this alone does NOT touch NVM or consume a write cycle -- it's
     * only the later Copy NV Block command that does). */
    st = commstat_write(hi2c, COMMSTAT_UNLOCK, 2);
    if (st != MAX17320_OK) {
        return st;
    }

    for (size_t i = 0; i < MAX17320_TARGET_CONFIG_COUNT; i++) {
        st = max17320_write_reg(hi2c, max17320_target_config[i].addr, max17320_target_config[i].value);
        if (st != MAX17320_OK) {
            commstat_write(hi2c, COMMSTAT_LOCK, 2); /* best-effort re-lock before bailing */
            return st;
        }
        HAL_Delay(2);
    }

    /* Re-lock write protection; not committing NVM here so this is just
     * good hygiene, not required for correctness. */
    return commstat_write(hi2c, COMMSTAT_LOCK, 2);
}

max17320_status_t max17320_verify_shadow_config(I2C_HandleTypeDef *hi2c,
                                                 size_t *mismatches,
                                                 size_t mismatch_cap,
                                                 size_t *mismatch_count)
{
    size_t found = 0;

    for (size_t i = 0; i < MAX17320_TARGET_CONFIG_COUNT; i++) {
        uint16_t readback = 0;
        max17320_status_t st = max17320_read_reg(hi2c, max17320_target_config[i].addr, &readback);
        if (st != MAX17320_OK) {
            return st;
        }
        if (readback != max17320_target_config[i].value) {
            if (mismatches != NULL && found < mismatch_cap) {
                mismatches[found] = i;
            }
            found++;
        }
    }

    if (mismatch_count != NULL) {
        *mismatch_count = found;
    }
    return (found == 0) ? MAX17320_OK : MAX17320_ERR_MISMATCH;
}

max17320_status_t max17320_read_remaining_nvm_updates(I2C_HandleTypeDef *hi2c,
                                                       uint8_t *used,
                                                       uint8_t *remaining)
{
    max17320_status_t st;
    uint16_t flags = 0;

    /* 1. unlock */
    st = commstat_write(hi2c, COMMSTAT_UNLOCK, 2);
    if (st != MAX17320_OK) return st;

    /* 2. request remaining-config-writes recall */
    st = max17320_write_reg(hi2c, REG_COMMAND, CMD_RECALL_REMAINING_UPDATES);
    if (st != MAX17320_OK) return st;

    /* 3. wait tRECALL */
    HAL_Delay(T_RECALL_MS);

    /* 4. read the indicator flags */
    st = max17320_read_reg(hi2c, REG_NV_REMAINING_FLAGS, &flags);
    if (st != MAX17320_OK) return st;

    /* 5. OR upper/lower byte, popcount = updates used (datasheet p.129) */
    {
        uint8_t or_byte = (uint8_t)((flags & 0xFFu) | (flags >> 8));
        uint8_t count = 0;
        for (uint8_t b = 0; b < 8; b++) {
            if (or_byte & (1u << b)) count++;
        }
        if (used != NULL) *used = count;
        if (remaining != NULL) *remaining = (count >= 7u) ? 0u : (uint8_t)(7u - count);
    }

    /* 6. lock write protection again */
    return commstat_write(hi2c, COMMSTAT_LOCK, 2);
}

#ifndef MAX17320_I_KNOW_THIS_BURNS_NVM
#define MAX17320_I_KNOW_THIS_BURNS_NVM 0
#endif

max17320_status_t max17320_commit_nvm(I2C_HandleTypeDef *hi2c, uint32_t confirm_token)
{
#if MAX17320_I_KNOW_THIS_BURNS_NVM
    max17320_status_t st;
    uint16_t commstat = 0;

    if (confirm_token != MAX17320_NVM_CONFIRM_TOKEN) {
        return MAX17320_ERR_NOT_CONFIRMED;
    }

    /* Steps 1+3: unlock write protection, then clear NVError (3x total per
     * datasheet's "Nonvolatile Block Programming" sequence). Step 2 (write
     * desired shadow values) must already have been done by the caller via
     * max17320_write_shadow_config()+verify. */
    printf("  [commit] step 1+3: unlock write protection (3x)...\r\n");
    st = commstat_write(hi2c, COMMSTAT_UNLOCK, 3);
    if (st != MAX17320_OK) { printf("  [commit] FAILED at step 1+3, status=%d\r\n", (int)st); return st; }

    /* Step 4: initiate block copy */
    printf("  [commit] step 4: Copy NV Block (0xE904)...\r\n");
    st = max17320_write_reg(hi2c, REG_COMMAND, CMD_COPY_NV_BLOCK);
    if (st != MAX17320_OK) { printf("  [commit] FAILED at step 4, status=%d\r\n", (int)st); return st; }

    /* Step 5: wait tBLOCK */
    printf("  [commit] step 5: waiting tBLOCK (%lu ms)...\r\n", (unsigned long)T_BLOCK_MS);
    HAL_Delay(T_BLOCK_MS);

    /* Step 6: check NVError */
    printf("  [commit] step 6: checking CommStat.NVError...\r\n");
    st = max17320_read_reg(hi2c, REG_COMMSTAT, &commstat);
    if (st != MAX17320_OK) { printf("  [commit] FAILED at step 6 (read CommStat), status=%d\r\n", (int)st); return st; }
    if (commstat & COMMSTAT_NVERROR_MASK) {
        printf("  [commit] NVError SET (CommStat=0x%04X) -- block copy did not complete successfully.\r\n", commstat);
        /* Do NOT auto-retry: each attempt consumes one of the 7 lifetime
         * writes. Leave the device as-is and let the caller decide,
         * re-running max17320_read_remaining_nvm_updates() first. */
        return MAX17320_ERR_NVM_ERROR;
    }
    printf("  [commit] NVError clear (CommStat=0x%04X), block copy OK so far.\r\n", commstat);

    /* Step 7: full/hardware reset so new NV settings take effect */
    printf("  [commit] step 7: full/hardware reset (0x000F)...\r\n");
    st = max17320_write_reg(hi2c, REG_COMMAND, CMD_FULL_RESET);
    if (st != MAX17320_OK) { printf("  [commit] FAILED at step 7, status=%d\r\n", (int)st); return st; }

    /* Step 8: wait for IC reset (write protection resets with it) */
    HAL_Delay(10);

    printf("  [commit] Copy NV Block done (NVError clear) -- data is committed. "
           "Running post-write reset housekeeping (steps 9-12, no further NVM write)...\r\n");
    return max17320_finish_post_commit_reset(hi2c);
#else
    (void)hi2c;
    (void)confirm_token;
    return MAX17320_ERR_NOT_IMPLEMENTED;
#endif
}

/*
 * Steps 9-12 of the datasheet's "Nonvolatile Block Programming" sequence:
 * re-unlock, pulse Config2.POR_CMD to reset the fuel-gauge algorithm state
 * (NOT another NVM write), wait for it to clear, re-lock write protection.
 * Safe to call on its own and to retry as many times as needed -- it does
 * not touch nonvolatile memory or consume any of the 7 lifetime writes.
 * Split out from max17320_commit_nvm() so a timeout/glitch here never
 * requires repeating the actual Copy NV Block step.
 */
max17320_status_t max17320_finish_post_commit_reset(I2C_HandleTypeDef *hi2c)
{
#if MAX17320_I_KNOW_THIS_BURNS_NVM
    max17320_status_t st;
    uint16_t config2 = 0;
    uint32_t waited_ms;

    /* Step 9: unlock again */
    printf("  [reset] step 9: unlock write protection again (2x)...\r\n");
    st = commstat_write(hi2c, COMMSTAT_UNLOCK, 2);
    if (st != MAX17320_OK) { printf("  [reset] FAILED at step 9, status=%d\r\n", (int)st); return st; }

    /* Step 10: reset firmware via Config2.POR_CMD */
    printf("  [reset] step 10: Config2.POR_CMD...\r\n");
    st = max17320_write_reg(hi2c, REG_CONFIG2, CONFIG2_POR_CMD);
    if (st != MAX17320_OK) { printf("  [reset] FAILED at step 10, status=%d\r\n", (int)st); return st; }

    /* Step 11: wait for Config2.POR_CMD (bit15) to clear */
    printf("  [reset] step 11: waiting for POR_CMD to clear (up to %lu ms)...\r\n",
           (unsigned long)T_POR_POLL_TIMEOUT_MS);
    for (waited_ms = 0; waited_ms < T_POR_POLL_TIMEOUT_MS; waited_ms += 5) {
        st = max17320_read_reg(hi2c, REG_CONFIG2, &config2);
        if (st != MAX17320_OK) { printf("  [reset] FAILED at step 11 (read Config2), status=%d\r\n", (int)st); return st; }
        if ((config2 & CONFIG2_POR_CMD_BIT) == 0) {
            break;
        }
        HAL_Delay(5);
    }
    if (config2 & CONFIG2_POR_CMD_BIT) {
        printf("  [reset] step 11 TIMEOUT: Config2=0x%04X still shows POR_CMD set after %lu ms\r\n",
               config2, (unsigned long)T_POR_POLL_TIMEOUT_MS);
        return MAX17320_ERR_I2C; /* POR never completed within timeout -- safe to retry, no data was written here */
    }
    printf("  [reset] step 11 OK, Config2=0x%04X\r\n", config2);

    printf("  [reset] step 12: lock write protection...\r\n");

    /* Step 12: lock write protection */
    return commstat_write(hi2c, COMMSTAT_LOCK, 2);
#else
    (void)hi2c;
    return MAX17320_ERR_NOT_IMPLEMENTED;
#endif
}
