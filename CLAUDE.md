# uMyo firmware

nRF52-based firmware for the uMyo EMG sensor device.
Handles EMG acquisition, IMU (ACC/GYRO), MAG (QMC5883P/L), BLE GATT, nRF24 radio,
and the UF1 transport protocol for streaming sensor data to phone or base station.

## Active working directory

`uMyo_fw_v3_1/` — this is the current firmware. Work here.
Older versioned directories in the repo are historical, leave them alone.

## Build

```bash
cd uMyo_fw_v3_1
make clean
make -j
```

Ubuntu 22.04, nRF5 SDK with ARM GCC toolchain.
`make clean` before `make` is a safe habit, especially after any urf_lib changes.

## Active branch

`feature/uf1-phone-mode` — this is the current working branch, not merged to main.
`main` is not current. Always check branch before starting work.
Goal: get this reviewed and merged, then create GitHub releases with .hex files.

## urf_lib submodule

`urf_lib` is a submodule providing BLE and radio support.
After any changes to urf_lib: `make clean` before rebuilding firmware.
See `urf_lib/CLAUDE.md` for critical notes, especially the `BLE_FORCE_SAME_EVENT_RESPONSE` fix.

## Firmware variants (what gets flashed)

| Variant | Description |
|---|---|
| `fw-base` | MAG + memory fixes only. No BLE, no OTA. Needs SWD programmer to update. |
| `fw-ble` | BLE working, old bootloader. Needs one-time SWD flash to get OTA bootloader, then wireless forever. |
| `fw-ble-ota` | Full new stack, OTA-capable out of the box. This is the target state. |

Reference compiled `.hex` and `.bin` are in `fw-ble-ota-shipped-with-ukrposhta-march-2026/` — not a repo, just reference artifacts.

## LED indicators

- Blue → BLE mode
- Green → base (nRF24) mode
- Purple → nRF24 active

## UF1 transport — profile summary

One UF1 frame = one physical device. Raw EMG is canonical. FFT is computed locally, not transmitted.

| Profile | Content | Payload |
|---|---|---|
| S1 | raw EMG + QUAT | 60-byte notify |
| S2 | raw EMG + IMU + MAG + QUAT | 52-byte raw notify + 26-byte aux notify |
| M1 | multi-sensor raw only | bridge mode, not yet implemented in app |
| M2 | multi-sensor raw + one reference IMU | not yet implemented |
| X1 | full experimental, high-MTU | not promised |

MTU 64 = conservative baseline. MTU 247 = optimized, not universal.

## Flash memory layout

nRF52832, 512KB total flash.

```
0x00000 ┌────────────────────────────┐
        │  Bootloader                │  16 KB  (0x0000–0x3FFF)
0x04000 ├────────────────────────────┤  ← firmware starts here (FLASH ORIGIN)
        │  .text (code + rodata)     │  ~48 KB (ends ~0xF9D8 currently)
0x0FA00 ├────────────────────────────┤
        │  .data init image          │  ~256 B (ends ~0xFACC)
0x0FB00 ├────────────────────────────┤
        │  (free, ~27 pages)         │
0x2B000 ├────────────────────────────┤
        │  Calibration storage       │  4 KB — wear-leveled, 256 × 16-byte records
        │  (IMU zeroes, MAG min/max) │  ← STORAGE_PAGE_ADDRESS
0x2C000 ├────────────────────────────┤
        │  (free)                    │
        │  ...                       │
0x3FFFF ├────────────────────────────┤  ← OTA erase ceiling (max firmware = 0x3BFFF)
0x40000 ├────────────────────────────┤  ← safe zone — NEVER touched by OTA
        │  Device name storage       │  wear-leveled, 16-byte records
        │                            │  ← NAME_STORAGE_PAGE_ADDRESS
        │  (free above)              │
0x7FFFF └────────────────────────────┘
```

**Critical:** OTA erase is hardcapped at page 64 (0x40000). Everything at 0x40000 and above
survives firmware updates unconditionally.

**Calibration at 0x2B000 is safe while firmware stays under ~156KB.** Current firmware is
~48KB — plenty of headroom. But it is not protected by design, only by size. If firmware
ever grows significantly, calibration must be migrated to 0x40000+.
Do not place any new persistent storage below 0x40000.

## BLE GATT characteristics

Service UUID: `93375900-F229-8B49-B397-44B5899B8601`

| Characteristic | UUID | Handles | Properties | Purpose |
|---|---|---|---|---|
| Telemetry | `FC7A850D-C1A5-F61F-0DA7-9995621FBD01` | 0x31/0x32/0x33 | READ, NOTIFY | EMG/IMU/QUAT stream |
| Device name | `FC7A850D-C1A5-F61F-0DA7-9995621FBD02` | 0x34/0x35/0x36 | READ, WRITE_NO_RESP, NOTIFY | Read/write 16-byte device name |

**Device name characteristic behavior:**
- On connect: firmware sends one-shot notify with current name after CCCD enabled
- Android should READ the characteristic on connect — do not rely solely on the notify
- On write: firmware validates printable ASCII (0x20–0x7E), saves to flash, updates characteristic
- Name persists across power cycles and OTA updates (stored at 0x40000)
- Default if blank: `uMyo-XXYY` where XX/YY are last 2 bytes of chip MAC
- Max 15 usable chars + null terminator (16 bytes total)
- First 7 chars of name also appear in BLE advertising packet

## Device naming

Names are stored on the device itself — they survive OTA, are visible in BLE advertising,
and travel with the hardware regardless of which app or tool connects to it.

Same name = same group. Three devices all named `L-Forearm` form a group automatically.
Channel within group is assigned by connection order in the app.

Python workbench reads name from stream and labels lanes as `L-Forearm / Ch1` etc.
Renaming is done from the Android app only — Python is a read-only consumer of the name.

## ⚠️ Critical fix — BLE connected streaming

`umyo_ble_conn_tick(ms)` must run **unconditionally** while connected over GATT.
It must NOT be gated by `unsent_cnt` or any similar send-queue check.

This was the root cause of "notifications enabled, waiting for data..." with no stream arriving.
The ADV/legacy path may stay gated separately — that's fine.
Do not re-gate the connected tick call.

## ⚠️ Critical fix — QMC5883P MAG re-arm

In `qmc_read()`, after reading MAG data, the next read must be re-armed:
```c
twim_read_buf(qmc_addr, qmc_data_reg, 6);
```
Without this, the driver re-parses stale data on every call.
The fix is in place — do not remove it.

Also note: QMC5883L and QMC5883P are separate chips with separate code paths. Keep them distinct.
The old fallback path also exists. Don't accidentally merge these.

## S2 payload contract

- 52 bytes = raw3 (EMG from 3 electrodes)
- 26 bytes = aux26 (IMU + MAG + QUAT)

Android parses these separately. If payload sizes change, Android parser must change too — coordinate.

## What's not yet done

- Multi-device BLE — single-device only right now, active next work area
- Android app refactor for multi-GATT + device naming UI (in progress)
- Python workbench update to read name from stream and label lanes
- Battery % not yet filled in STATUS frames
- Calibration storage migration to 0x40000+ (low urgency while firmware < 156KB)
- Branches not yet merged, no GitHub releases yet
