/*
 * canbus.c
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 */
#include "canbus.h"

static const char *TAG_CAN = "CANBUS";

static QueueHandle_t scan_ready_queue = NULL;   // length-1, overwrite -- always the latest scan
bool canbus_last_publish_ok = false;

/**
 * @brief Initialize the CAN (TWAI) driver: 500kbit/s, normal mode, accept-all filter (this
 *        node only ever transmits, but we install a normal-mode driver so a future firmware
 *        revision can add bus diagnostics/listen without a driver-mode change).
 */
esp_err_t canbus_init(void)
{
	printf("Initializing CAN bus (broadcast publisher)...\n");

	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_CAN, "twai_driver_install failed: %s", esp_err_to_name(err));
		return err;
	}

	err = twai_start();
	if (err != ESP_OK) {
		ESP_LOGE(TAG_CAN, "twai_start failed: %s", esp_err_to_name(err));
		return err;
	}

	scan_ready_queue = xQueueCreate(1, sizeof(rtd_scan_result_t));
	if (scan_ready_queue == NULL) {
		ESP_LOGE(TAG_CAN, "Failed to create scan-ready queue");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG_CAN, "CAN bus ready at 500kbit/s, broadcasting IDs 0x%03X-0x%03X", CAN_ID_TEMP_GROUP_BASE, CAN_ID_STATUS);
	return ESP_OK;
}

void canbus_notify_new_scan(const rtd_scan_result_t *scan)
{
	if (scan_ready_queue == NULL) return;
	xQueueOverwrite(scan_ready_queue, scan);
}

static bool transmit_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
	twai_message_t msg = {
		.identifier = id,
		.extd = 0,
		.rtr = 0,
		.data_length_code = dlc,
	};
	memcpy(msg.data, data, dlc);

	// No-ack, single-shot: this is a broadcast, nobody on this bus is expected to answer,
	// and we are not going to retry a stale reading into the next scan period.
	msg.ss = 1;

	esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(50));
	if (err != ESP_OK) {
		ESP_LOGW(TAG_CAN, "CAN transmit 0x%03lX failed: %s", (unsigned long)id, esp_err_to_name(err));
		return false;
	}
	return true;
}

/**
 * @brief Publish one 16-channel scan as a 5-frame burst (4 temperature groups + 1 status frame).
 */
static bool publish_scan(const rtd_scan_result_t *scan)
{
	bool all_ok = true;

	for (int group = 0; group < 4; group++) {
		uint8_t payload[8];
		for (int i = 0; i < CAN_CHANNELS_PER_FRAME; i++) {
			int ch = group * CAN_CHANNELS_PER_FRAME + i;
			int16_t scaled = (int16_t)(scan->temperature_c[ch] * 100.0f);
			payload[i * 2]     = scaled & 0xFF;          // little-endian
			payload[i * 2 + 1] = (scaled >> 8) & 0xFF;
		}
		all_ok &= transmit_frame(CAN_ID_TEMP_GROUP_BASE + group, payload, 8);
		vTaskDelay(pdMS_TO_TICKS(CAN_FRAME_GAP_MS));
	}

	uint16_t validity_bitmap = 0, fault_bitmap = 0;
	uint8_t health = 0;
	for (int ch = 0; ch < RTD_CHANNEL_COUNT; ch++) {
		if (scan->valid[ch]) validity_bitmap |= (1 << ch);
		if (scan->fault[ch] != RTD_FAULT_NONE) fault_bitmap |= (1 << ch);
	}
	health |= (all_ok ? (1 << 0) : 0);

	uint8_t status_payload[8] = {
		(uint8_t)(scan->sequence & 0xFF), (uint8_t)((scan->sequence >> 8) & 0xFF),
		(uint8_t)(validity_bitmap & 0xFF), (uint8_t)((validity_bitmap >> 8) & 0xFF),
		(uint8_t)(fault_bitmap & 0xFF), (uint8_t)((fault_bitmap >> 8) & 0xFF),
		health, 0x00,
	};
	all_ok &= transmit_frame(CAN_ID_STATUS, status_payload, 8);

	return all_ok;
}

/**
 * @brief Body of the dedicated CAN task: block on the acquisition task's scan-ready queue,
 *        publish, repeat. Runs completely decoupled from the Modbus task -- a slow or blocked
 *        CAN transmit only delays this task, never the RS485/Modbus response path.
 */
void canbus_publish_task_body(void)
{
	rtd_scan_result_t scan;
	if (xQueueReceive(scan_ready_queue, &scan, portMAX_DELAY) == pdTRUE) {
		canbus_last_publish_ok = publish_scan(&scan);
		if (!canbus_last_publish_ok) {
			ESP_LOGW(TAG_CAN, "One or more frames in this scan's CAN burst failed to send");
		}
	}
}
