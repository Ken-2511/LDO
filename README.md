# Stackable BLE low-noise power supply (PoC)

Two KiCad 10 projects, stacked on the same 40x40 mm footprint:

- `controller/`: Seeed XIAO nRF52840 board at the bottom of the stack. It supplies 5V (from USB) and 3.3V, and carries the I2C pull-ups.
- `ldo/`: LDO board, one LT3042 channel set by a DAC60502 and read back by an INA226. Up to 4 boards stack on the controller, each on its own I2C address.

The boards connect through a 2x5 stacking header (J1) at the same position on every board:
pins 1-2 +5V, pin 3 +3V3, pin 5 SDA, pin 7 SCL, pins 4, 6, 8, 9, 10 GND.
