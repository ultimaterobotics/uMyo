# uMyo firmware

Firmware for uMyo — a wireless wearable single-channel EMG sensor with IMU and magnetometer, built on nRF52832.

**Buy assembled devices:** https://udevices.io/products/umyo-wearable-emg-sensor
**Docs:** https://make.udevices.io
**Discord:** https://discord.com/invite/dEmCPBzv9G

## What's new — March 2026

- **BLE streaming via Android bridge** — uMyo streams EMG + IMU + magnetometer over BLE to an Android phone, which forwards data over WiFi to the PC workbench
- **Multi-device** — 3 simultaneous uMyos confirmed working
- **OTA firmware updates** — wireless updates via Android app (requires OTA-capable bootloader)
- **New web workbench** — browser-based GUI with per-device waveform, spectrum, 3D orientation, ACC/GYRO sparklines

See [What's new — March 2026](https://make.udevices.io/guides/whats-new-march-2026) for the full update.

## Repository structure

```
uMyo_fw_v2_1/                  older hardware revision
uMyo_fw_v3_1/                  current hardware — nRF24/USB base mode
uMyo_fw_v3_1_April_2025_RGB_update/  RGB LED update
urf_lib/                       submodule — nRF52 BLE stack and helpers
```

The new BLE/OTA firmware (`feature/uf1-phone-mode` branch) is pending merge to main — see [uf1-tools](https://github.com/ultimaterobotics/uf1-tools) and [umyo-android](https://github.com/ultimaterobotics/umyo-android) for the full stack.

## Related repositories

- [uf1-tools](https://github.com/ultimaterobotics/uf1-tools) — Python + browser workbench for receiving and visualizing data
- [umyo-android](https://github.com/ultimaterobotics/umyo-android) — Android BLE bridge app
- [umyo-bootloader](https://github.com/ultimaterobotics/umyo-bootloader) — bootloader with OTA support
- [uMyo_v3_1_pcb](https://github.com/ultimaterobotics/uMyo_v3_1_pcb) — PCB design
- [urf_lib](https://github.com/ultimaterobotics/urf_lib) — nRF52 micro SDK (shared across uDevices projects)
- [uMyo_RF24](https://github.com/ultimaterobotics/uMyo_RF24) — Arduino library for nRF24 radio mode
- [uMyo_BLE](https://github.com/ultimaterobotics/uMyo_BLE) — Arduino library for ESP32 BLE mode
- [uMyo_python_tools](https://github.com/ultimaterobotics/uMyo_python_tools) — Python tools for USB base station mode

## Building

Requires `arm-none-eabi-gcc` and the `urf_lib` submodule.

```bash
git clone https://github.com/ultimaterobotics/uMyo.git
cd uMyo
git submodule update --init --recursive
```

Tested with GNU Arm Embedded Toolchain 8-2018-q4-major (GCC 8.2.1).

```bash
# Ubuntu / Debian
sudo apt install gcc-arm-none-eabi
```

## Flashing

**Wireless (OTA)** — requires OTA-capable bootloader. Use the Android app. See [OTA guide](https://make.udevices.io/guides/whats-new-march-2026#how-to-update-firmware-via-ota).

**Wired (SWD)** — ST-Link v2 clone, Raspberry Pi Pico (picoprobe), or any CMSIS-DAP probe + openocd. See [SWD flashing guide](https://make.udevices.io/guides/firmware-flash-swd).

## License

MIT
