# uMyo device

This repository contains firmware only, other repositories:
* PCB design for v3.1: https://github.com/ultimaterobotics/uMyo_v3_1_pcb/
* PCB design for v2: https://github.com/ultimaterobotics/uMyo_v2_pcb
* Tools for receiving+processing data in Python: https://github.com/ultimaterobotics/uMyo_python_tools
* Arduino library for receiving data via nRF24 radio: https://github.com/ultimaterobotics/uMyo_RF24
* Arduino library for receiving data via ESP32 BLE: https://github.com/ultimaterobotics/uMyo_BLE
* nRF52x handling library and micro SDK: https://github.com/ultimaterobotics/urf_lib
* Bootloader is the same as for uECG device https://github.com/ultimaterobotics/uECG_bootloader - but with pin numbers adjusted for uMyo board

uMyo is a wireless wearable EMG sensor with on-board IMU. By default it uses dry contact electrodes - which can be replaced with connectors to standard gel electrodes for higher signal quality. It acquires data, performs 8-bin FFT and calculates muscle activity level based on presence of high~ish frequencies in the signal. It is not completely immune to 50/60Hz noise but filters out a significant part of it, so can be used in many practical cases.

Measured data are sent via radio in 3 modes (selected via button). One is compatible with popular Arduino nRF24 radio (although that somewhat reduces amount of data which can be sent), another requires nRF5x radio but all the measured data are sent, and yet another mode uses generic BLE advertisement - which significantly limits bandwidth but still can be reasonably used with ESP32's BLE).

Design and firmware are open source with permissive licenses, and also we sell assembled devices on Tindie: https://www.tindie.com/products/28529/

For building it requires urf_lib and arm-none-eabi compiler.

Important code properties:
 - ADC reading using DMA, FFT calculations in a way that won't interrupt data acquisition process (handled in adc_read.c)
 - IMU data integration into orientation quaternion (handled in lsm6ds3.c)
 - Filtering 50/60 Hz noise and sending data via one of 3 configurable radio interfaces (main.c, functions push_adc_data(), prepare_and_send_BLE(), prepare_data_packet32() for nRF24 mode, prepare_data_packet() for base station mode)
