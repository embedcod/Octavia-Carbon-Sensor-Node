# Octavia Carbon -- Sensor Node Firmware (On-Site POC Build)

Stripped-down FreeRTOS (ESP-IDF) firmware for proving the acquisition -> Modbus
and acquisition -> CAN pipeline on Octavia Carbon's own hardware. No calibration,
no NVS, no button/LED -- three tasks, one job each.

## Build

```
idf.py set-target esp32
idf.py build
```

## What changed from the design-challenge submission

Removed entirely, since this build's only job is proving the pipeline works:

- NVS-backed per-channel calibration (`rtd_calibration_t`, `rtd_calibration_load_all`,
  `rtd_calibration_set`) -- `rtd_scan_all_channels()` now returns the raw
  Callendar-Van Dusen conversion, uncorrected.
- Modbus holding registers / FC06 write path (they existed only to carry
  calibration writes) -- this build is Modbus **read-only**. Both FC03 and FC04
  answer the same 20-register table, since some PLC master configurations
  default to FC03 and there's no separate holding-register block to send them to
  anymore.
- Button task, status LED, debounce/multi-press logic, `POWER_PIN` shutdown line.
- `RS485_EN_PIN` and `CAN_EN_PIN` power-switching GPIOs -- Octavia's hardware
  doesn't have these enable lines, so `modbus_slave_init()` and `canbus_init()`
  no longer touch any GPIO beyond the bus pins themselves.
- `main.c` creates exactly three tasks: `acquisition_task` (Core 1),
  `modbus_task` (Core 0), `can_task` (Core 0). No NVS init in `app_main()`.

Unchanged, on purpose:

- The RTD-type selection macro is exactly as you wrote it in your own edit --
  one `#define RTD_SENSOR_TYPE` line, everything else (`RTD_NOMINAL_OHMS`,
  `RTD_RREF_OHMS`) derives from it via `#if`. Octavia's hardware uses the same
  4.3k/430R jumper-selection approach, so nothing about this needed to change.
- Modbus register map (`modbus.h`), CAN frame catalogue (`canbus.h`), and the
  three-task concurrency model (acquisition owns SPI/I2C and never blocks
  Modbus or CAN) are unchanged from what the two ICDs already document. You
  should only need to revisit ICD wording if register/frame addresses actually
  move -- they haven't.

## Chip-select hardware change: two PCF8574 -> one MCP23017

Octavia's board uses a single MCP23017T-E/SO instead of your two PCF8574s.
`i2c_bus.c` is rewritten around it:

- MCP23017 is a register-addressed device (write `[register, value]`), not a
  bare-byte device like the PCF8574 -- every access in the new `i2c_bus.c` goes
  through `mcp23017_write_reg(reg, value)`.
- Port A (`GPA0-7`) drives CS1-CS8, Port B (`GPB0-7`) drives CS9-CS16 --
  **confirm this A/B split against the actual board**, the same way the two
  PCF8574 addresses needed confirming before. I don't have a way to verify pin
  ordering from a part number alone.
- Address assumed `0x20` (all address pins to GND, the MCP23017 default) --
  **confirm against the board's strapping**. If it's anything else, it's one
  line to change: `MCP23017_ADDR` in `pinout.h`.
- `spi_bus.c`'s calls into `cs_expander_select()`/`cs_expander_deselect_all()`
  are unchanged -- the MCP23017 swap is entirely contained in `i2c_bus.c`, none
  of the SPI/MAX31865 logic needed to know or care which expander chip is behind
  those two functions.

CS1 -> MAX31865 #1 (RTD channel 1) ... CS16 -> MAX31865 #16 (RTD channel 16),
matching what you confirmed on Octavia's hardware.

## On-site checklist for tomorrow

1. Confirm `MCP23017_ADDR` (0x20 assumed) and the Port A/B -> CS1-16 mapping
   against the physical board before powering the RTD bus.
2. Flash, open the serial monitor -- `acquisition_task` prints all 16 channel
   readings, validity, and fault flags to the console every ~2s regardless of
   whether anything is polling Modbus or CAN, so you can confirm real
   temperatures are coming back before you even bring up the PLC or the CM5.
3. Point the PLC (or a Modbus test master/laptop tool) at slave address 1,
   9600 baud (matches `pinout.h` as you set it), FC04 (or FC03, both work),
   registers 30001-30020. Confirm it matches the console output.
4. Point a CAN sniffer or the CM5's CAN interface at IDs 0x150-0x154 and
   confirm the 5-frame burst every ~2s matches the same console output.
5. Both ICDs (Modbus, CAN) you already have describe exactly this register map
   and frame catalogue -- they didn't need to change for this build, so use
   them as-is for the PLC and CM5 work tomorrow.
