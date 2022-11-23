# Firmware source code for uMyo device

For building it requires our library https://github.com/ultimaterobotics/urf_lib and arm-none-eabi compiler.
Firmware upload can be performed via st-link or wirelessly using our base station device (or other nRF52 compatible radio). Bootloader is essentially the same as https://github.com/ultimaterobotics/uECG_bootloader with pin numbers tailored for uMyo board.

Major code parts:
 - ADC reading using DMA, FFT calculations in a way that won't interrupt data acquisition process (handled in adc_read.c)
 - IMU data integration into orientation quaternion (handled in lsm6ds3.c)
 - Filtering 50/60 Hz noise and sending data via one of 3 configurable radio interfaces (main.c, functions push_adc_data(), prepare_and_send_BLE(), prepare_data_packet32() for nRF24 mode, prepare_data_packet() for base station mode)
