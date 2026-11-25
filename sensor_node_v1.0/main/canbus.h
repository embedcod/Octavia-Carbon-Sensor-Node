/*
 * canbus.h
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 *
 *
 *  Frame catalogue (multiplexed - 16 channels x int16 do not fit one
 *  classic 8-byte CAN frame):
 *
 *    0x150  Channels 1-4   temperature, 4x int16 (x100 deg C), little-endian
 *    0x151  Channels 5-8   temperature, 4x int16 (x100 deg C), little-endian
 *    0x152  Channels 9-12  temperature, 4x int16 (x100 deg C), little-endian
 *    0x153  Channels 13-16 temperature, 4x int16 (x100 deg C), little-endian
 *    0x154  Status: [0-1] sequence (u16) [2-3] validity bitmap (u16)
 *                   [4-5] fault bitmap (u16) [6] board health (u8) [7] reserved
 *
 *  IDs chosen mid-range (0x150-0x154) so this telemetry burst does not
 *  starve out anything more time-critical that shares the bus in a future
 *  revision; the full bus-loading calculation belongs in the Part A ICD.
 */

#ifndef MAIN_CANBUS_H_
#define MAIN_CANBUS_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "pinout.h"
#include "spi_bus.h"

#define CAN_ID_TEMP_GROUP_BASE   0x150   // + 0,1,2,3 for channel groups 1-4, 5-8, 9-12, 13-16
#define CAN_ID_STATUS            0x154
#define CAN_CHANNELS_PER_FRAME   4
#define CAN_FRAME_GAP_MS         5       // Small inter-frame gap so the 5-frame burst doesn't look like a babble

esp_err_t canbus_init(void);
// Blocking: waits on the acquisition task's scan-ready queue and publishes one full 5-frame
// burst per scan. 
void canbus_publish_task_body(void);
// Called by the acquisition task once per completed scan to hand data to the CAN task.
void canbus_notify_new_scan(const rtd_scan_result_t *scan);
extern bool canbus_last_publish_ok;

#endif /* MAIN_CANBUS_H_ */
