/*
 * i2c_bus.h
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 *
 *  Octavia Carbon on-site hardware. I2C master driver: the only device on this bus is
 *  a single MCP23017 16-bit GPIO expander, used exclusively to fan out the 16 chip-select
 *  lines for the MAX31865 RTD front ends on the SPI bus (spi_bus.c calls into this driver
 *  before/after every per-channel transaction). Port A (GPA0-7) drives CS1-CS8, Port B
 *  (GPB0-7) drives CS9-CS16 -- CONFIRM this A/B split against the board before relying on it.
 */

#ifndef MAIN_I2C_BUS_H_
#define MAIN_I2C_BUS_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "pinout.h"

/**********************************************************************************************************************/
#define CS_EXPANDER_CHANNEL_COUNT   16
/**********************************************************************************************************************/

void i2c_scan();
esp_err_t i2c_bus_init(void);

// CS-expander channel control: exactly one of the 16 CS lines is driven low, the rest held high.
// This is what spi_bus.c wraps around every MAX31865 register transaction.
esp_err_t cs_expander_select(uint8_t channel);   // channel: 0-15 -> pull CS(channel+1) low, all others high
esp_err_t cs_expander_deselect_all(void);        // idle state: all 16 CS lines high


// Mutex guarding the MCP23017. spi_bus.c must hold this for the full duration of a channel's
// SPI transaction, not just the CS toggle -- otherwise a second task could reselect another
// channel mid-transfer.
extern SemaphoreHandle_t i2c_bus_mutex;

#endif /* MAIN_I2C_BUS_H_ */
