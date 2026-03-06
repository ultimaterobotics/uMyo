sudo openocd -f interface/stlink.cfg -f target/nrf52.cfg -c init -c "reset halt" -c "flash read_bank 0 build/dump.hex 0 0x30000" -c "reset" -c exit

