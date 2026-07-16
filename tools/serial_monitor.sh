#!/usr/bin/env bash
# Live 115200-baud serial view of the ST-Link Virtual COM Port (auto-detects the device).
# Exit: Ctrl+A then Ctrl+X.

port=$(find /dev/serial/by-id -iname '*stlink*' 2>/dev/null | head -1)
[ -z "$port" ] && port=$(ls /dev/ttyACM* 2>/dev/null | head -1)

if [ -z "$port" ]; then
    echo "ST-Link VCP not found — is the board plugged in?"
    exit 1
fi

echo "Connected to $port @ 115200 (exit: Ctrl+A, Ctrl+X)"
exec picocom -b 115200 --imap lfcrlf "$port"
