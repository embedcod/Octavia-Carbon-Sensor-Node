/*
 * modbus.h
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 *
 *  Octavia Carbon on-site POC. Modbus RTU SLAVE driver. The S7-1200 PLC is the Modbus
 *  master. Read-only in this build -- no calibration/holding-register path, this is a
 *  proof-of-concept pipeline test, not the full instrument.
 *
 *  INPUT REGISTERS (FC 0x04 -- FC 0x03 is also accepted and answers the same table,
 *  for master compatibility, since there is no separate holding-register block here):
 *    30001-30016  Channel 1-16 temperature, signed int16, x100 (2350 = 23.50C)
 *    30017        Per-channel validity bitmap (bit n = 1 -> channel n+1 is valid)
 *    30018        Per-channel fault-present bitmap (bit n = 1 -> channel n+1 has a fault flag set)
 *    30019        Scan sequence counter, low 16 bits (increments once per completed 16-ch scan)
 *    30020        Board health: bit0 SPI bus ok on last scan, bit1 I2C bus ok
 */

#ifndef MAIN_MODBUS_H_
#define MAIN_MODBUS_H_

#include "string.h"
#include "esp_log.h"
#include <stdio.h>
#include <esp_system.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "driver/uart.h"
#include "esp_err.h"
#include "pinout.h"
#include "spi_bus.h"

/********************************************************************************************************************
DEFINITIONS: MODBUS RTU SLAVE
****************************/
#define MODBUS_BUFF_SIZE            256
#define MODBUS_MAX_FRAME            256
#define MODBUS_RX_TIMEOUT           pdMS_TO_TICKS(200)   // Inter-character + response window
#define MODBUS_FC_READ_HOLDING      0x03
#define MODBUS_FC_READ_INPUT        0x04
#define MODBUS_EXCEPTION_BIT        0x80
#define MODBUS_EXC_ILLEGAL_FUNCTION 0x01
#define MODBUS_EXC_ILLEGAL_ADDRESS  0x02

#define MODBUS_INPUT_REG_COUNT      20   // 16 channels + validity + fault + sequence + health
/**********************************************************************************************************************/
uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
esp_err_t modbus_slave_init(void);

// Blocking: waits for one RTU frame addressed to us, services it, replies. Call in a tight
// loop from the dedicated Modbus task (main.c). This function owns the RS485 bus.
void modbus_slave_poll(void);

// Called by the acquisition task after every completed rtd_scan_all_channels() to publish
// fresh data into the input-register table. Takes modbus_data_mutex internally.
void modbus_slave_update_data(const rtd_scan_result_t *scan, bool spi_ok, bool i2c_ok);

extern SemaphoreHandle_t modbus_data_mutex;

#endif /* MAIN_MODBUS_H_ */
