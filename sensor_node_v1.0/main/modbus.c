/*
 * modbus.c
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 */
#include "modbus.h"

static const char *TAG_MB = "MODBUS_SLAVE";

SemaphoreHandle_t modbus_data_mutex = NULL;

static uint16_t input_regs[MODBUS_INPUT_REG_COUNT];     // 30001.. table, see modbus.h
static uint16_t holding_regs[MODBUS_HOLDING_REG_COUNT];  // 40001.. table, see modbus.h

/**
 * @brief Modbus RTU CRC16
 */
uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
	uint16_t crc = 0xFFFF;
	for (uint16_t pos = 0; pos < len; pos++) {
		crc ^= buf[pos];
		for (int i = 0; i < 8; i++) {
			if (crc & 0x0001) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

/**
 * @brief Initialize UART1 for RS485 half-duplex, slave side
 */
esp_err_t modbus_slave_init(void)
{
	printf("Initializing Modbus RTU slave (RS485)...\n");

	// Automatic RS485 power-switching enable
	esp_rom_gpio_pad_select_gpio(RS485_EN_PIN);
	gpio_set_direction(RS485_EN_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(RS485_EN_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(100));

	uart_config_t uart_config = {
		.baud_rate = MODBUS_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	esp_err_t err = uart_driver_install(MODBUS_UART_NUM, MODBUS_BUFF_SIZE * 2, 0, 0, NULL, 0);
	if (err != ESP_OK) return err;
	err = uart_param_config(MODBUS_UART_NUM, &uart_config);
	if (err != ESP_OK) return err;
	err = uart_set_pin(MODBUS_UART_NUM, MODBUS_TX_PIN, MODBUS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) return err;
	err = uart_set_mode(MODBUS_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
	if (err != ESP_OK) return err;

	modbus_data_mutex = xSemaphoreCreateMutex();
	if (modbus_data_mutex == NULL) {
		ESP_LOGE(TAG_MB, "Failed to create Modbus data mutex");
		return ESP_FAIL;
	}

	memset(input_regs, 0, sizeof(input_regs));
	memset(holding_regs, 0, sizeof(holding_regs));

	ESP_LOGI(TAG_MB, "Modbus RTU slave ready on UART%d, address %d, %d baud",
			 MODBUS_UART_NUM, MODBUS_SLAVE_ID, MODBUS_BAUD_RATE);
	return ESP_OK;
}

/**
 * @brief Publish a completed RTD scan into the input-register table (30001..)
 */
void modbus_slave_update_data(const rtd_scan_result_t *scan, bool spi_ok, bool i2c_ok, bool can_ok)
{
	if (xSemaphoreTake(modbus_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
		ESP_LOGW(TAG_MB, "modbus_slave_update_data: could not take data mutex, dropping this update");
		return;
	}

	uint16_t validity_bitmap = 0;
	uint16_t fault_bitmap = 0;
	bool any_uncalibrated = false;

	for (int ch = 0; ch < RTD_CHANNEL_COUNT; ch++) {
		int16_t scaled = (int16_t)(scan->temperature_c[ch] * 100.0f);
		input_regs[ch] = (uint16_t)scaled;

		if (scan->valid[ch]) validity_bitmap |= (1 << ch);
		if (scan->fault[ch] != RTD_FAULT_NONE) fault_bitmap |= (1 << ch);
		if (scan->fault[ch] & RTD_FAULT_UNCALIBRATED) any_uncalibrated = true;
	}

	input_regs[16] = validity_bitmap;
	input_regs[17] = fault_bitmap;
	input_regs[18] = (uint16_t)(scan->sequence & 0xFFFF);

	uint16_t health = 0;
	if (spi_ok) health |= (1 << 0);
	if (i2c_ok) health |= (1 << 1);
	if (any_uncalibrated) health |= (1 << 2);
	if (can_ok) health |= (1 << 3);
	input_regs[19] = health;

	xSemaphoreGive(modbus_data_mutex);
}

static void send_exception(uint8_t function, uint8_t exception_code)
{
	uint8_t tx[5];
	tx[0] = MODBUS_SLAVE_ID;
	tx[1] = function | MODBUS_EXCEPTION_BIT;
	tx[2] = exception_code;
	uint16_t crc = modbus_crc16(tx, 3);
	tx[3] = crc & 0xFF;
	tx[4] = crc >> 8;
	uart_write_bytes(MODBUS_UART_NUM, (const char *)tx, 5);
	uart_wait_tx_done(MODBUS_UART_NUM, pdMS_TO_TICKS(50));
}

static void handle_read_registers(uint8_t function, const uint8_t *rx)
{
	uint16_t start_addr = (rx[2] << 8) | rx[3];
	uint16_t qty = (rx[4] << 8) | rx[5];

	uint16_t *table = (function == MODBUS_FC_READ_INPUT) ? input_regs : holding_regs;
	uint16_t table_len = (function == MODBUS_FC_READ_INPUT) ? MODBUS_INPUT_REG_COUNT : MODBUS_HOLDING_REG_COUNT;

	if (qty == 0 || qty > 125 || (start_addr + qty) > table_len) {
		send_exception(function, MODBUS_EXC_ILLEGAL_ADDRESS);
		return;
	}

	uint8_t byte_count = qty * 2;
	uint8_t tx[5 + 250];
	tx[0] = MODBUS_SLAVE_ID;
	tx[1] = function;
	tx[2] = byte_count;

	if (xSemaphoreTake(modbus_data_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
		send_exception(function, MODBUS_EXC_ILLEGAL_ADDRESS);   // Best available exception; bus should not stall
		return;
	}
	for (int i = 0; i < qty; i++) {
		tx[3 + i * 2]     = (table[start_addr + i] >> 8) & 0xFF;
		tx[3 + i * 2 + 1] = table[start_addr + i] & 0xFF;
	}
	xSemaphoreGive(modbus_data_mutex);

	uint16_t crc = modbus_crc16(tx, 3 + byte_count);
	tx[3 + byte_count] = crc & 0xFF;
	tx[3 + byte_count + 1] = crc >> 8;

	uart_write_bytes(MODBUS_UART_NUM, (const char *)tx, 5 + byte_count);
	uart_wait_tx_done(MODBUS_UART_NUM, pdMS_TO_TICKS(100));
}

static void handle_write_single_register(const uint8_t *rx)
{
	uint16_t addr = (rx[2] << 8) | rx[3];
	uint16_t value = (rx[4] << 8) | rx[5];

	if (addr >= MODBUS_HOLDING_REG_COUNT) {
		send_exception(MODBUS_FC_WRITE_SINGLE, MODBUS_EXC_ILLEGAL_ADDRESS);
		return;
	}

	xSemaphoreTake(modbus_data_mutex, pdMS_TO_TICKS(100));
	holding_regs[addr] = value;
	xSemaphoreGive(modbus_data_mutex);

	// Register 40004 is the calibration-commit trigger - see modbus.h register map
	if (addr == 3 && value == 1) {
		uint8_t channel = (uint8_t)holding_regs[0];
		float offset_c = ((int16_t)holding_regs[1]) / 100.0f;
		float gain = holding_regs[2] / 10000.0f;
		esp_err_t err = rtd_calibration_set(channel, offset_c, gain);
		ESP_LOGI(TAG_MB, "Calibration commit ch=%u offset=%.2f gain=%.4f -> %s",
				 channel, offset_c, gain, esp_err_to_name(err));
		xSemaphoreTake(modbus_data_mutex, pdMS_TO_TICKS(100));
		holding_regs[3] = 0;   // Trigger always reads back 0 - see header comment
		xSemaphoreGive(modbus_data_mutex);
	}

	// Echo request per Modbus spec for FC06
	uint8_t tx[8];
	memcpy(tx, rx, 6);
	uint16_t crc = modbus_crc16(tx, 6);
	tx[6] = crc & 0xFF;
	tx[7] = crc >> 8;
	uart_write_bytes(MODBUS_UART_NUM, (const char *)tx, 8);
	uart_wait_tx_done(MODBUS_UART_NUM, pdMS_TO_TICKS(50));
}

/**
 * @brief Wait for and service exactly one RTU request addressed to this slave. Blocking with a
 *        bus-idle-style timeout - if nothing arrives, returns and the caller loops again. This
 *        never talks first: the PLC is always the initiator (Modbus master).
 */
void modbus_slave_poll(void)
{
	uint8_t rx[MODBUS_MAX_FRAME];
	int len = uart_read_bytes(MODBUS_UART_NUM, rx, MODBUS_MAX_FRAME, MODBUS_RX_TIMEOUT);

	if (len < 8) return;   // Nothing arrived, or too short to be a real request - just loop

	if (rx[0] != MODBUS_SLAVE_ID) return;   // Not addressed to us - another node's poll, ignore silently

	uint16_t crc_rx = (rx[len - 1] << 8) | rx[len - 2];
	uint16_t crc_calc = modbus_crc16(rx, len - 2);
	if (crc_rx != crc_calc) {
		ESP_LOGW(TAG_MB, "CRC mismatch on incoming frame, dropping (no reply -- master will retry)");
		return;
	}

	uint8_t function = rx[1];
	switch (function) {
		case MODBUS_FC_READ_INPUT:
		case MODBUS_FC_READ_HOLDING:
			handle_read_registers(function, rx);
			break;
		case MODBUS_FC_WRITE_SINGLE:
			handle_write_single_register(rx);
			break;
		default:
			send_exception(function, MODBUS_EXC_ILLEGAL_FUNCTION);
			break;
	}
}
