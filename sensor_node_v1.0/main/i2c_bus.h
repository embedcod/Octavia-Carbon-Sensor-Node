/*
 * i2c_bus.h
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
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

esp_err_t i2c_bus_init(void);
esp_err_t pcf8574_write_byte(uint8_t addr, uint8_t data);
esp_err_t pcf8574_read_byte(uint8_t addr, uint8_t *data);
esp_err_t cs_expander_select(uint8_t channel); 
esp_err_t cs_expander_deselect_all(void);    

extern SemaphoreHandle_t i2c_bus_mutex;

#endif /* MAIN_I2C_BUS_H_ */
