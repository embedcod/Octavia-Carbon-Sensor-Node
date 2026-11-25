/*
 * i2c_bus.c
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 */
#include "i2c_bus.h"

static const char *TAG_I2C = "I2C_BUS";

SemaphoreHandle_t i2c_bus_mutex = NULL;

static uint8_t cs_low_state  = 0xFF;   // channels 1-8   (PCF8574 @ PCF8574_CS_LOW_ADDR)
static uint8_t cs_high_state = 0xFF;   // channels 9-16  (PCF8574 @ PCF8574_CS_HIGH_ADDR)

/**
 * @brief Initialize the I2C master bus
 */
esp_err_t i2c_bus_init(void)
{
	printf("Initializing the I2C master...\n");

	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_SDA_PIN,
		.scl_io_num = I2C_SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_CLK_HZ,
	};

	esp_err_t err = i2c_param_config(I2C_PORT, &conf);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_I2C, "i2c_param_config failed: %s", esp_err_to_name(err));
		return err;
	}

	err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_I2C, "i2c_driver_install failed: %s", esp_err_to_name(err));
		return err;
	}

	i2c_bus_mutex = xSemaphoreCreateMutex();
	if (i2c_bus_mutex == NULL) {
		ESP_LOGE(TAG_I2C, "Failed to create I2C bus mutex");
		return ESP_FAIL;
	}

	// Force both expanders to the idle state (all CS high = all RTDs deselected) on boot,
	// before anything on the SPI bus assumes a particular channel is active.
	esp_err_t e1 = pcf8574_write_byte(PCF8574_CS_LOW_ADDR, 0xFF);
	esp_err_t e2 = pcf8574_write_byte(PCF8574_CS_HIGH_ADDR, 0xFF);
	if (e1 != ESP_OK || e2 != ESP_OK) {
		ESP_LOGE(TAG_I2C, "CS-expander idle-state init failed (low=%s, high=%s)",
				 esp_err_to_name(e1), esp_err_to_name(e2));
		return ESP_FAIL;
	}

	ESP_LOGI(TAG_I2C, "I2C bus and CS expanders ready, all 16 channels deselected");
	return ESP_OK;
}

/**
 * @brief Write a byte to a PCF8574 at the given 7-bit address
 */
esp_err_t pcf8574_write_byte(uint8_t addr, uint8_t data)
{
	return i2c_master_write_to_device(I2C_PORT, addr, &data, 1, pdMS_TO_TICKS(50));
}

/**
 * @brief Read the current input byte from a PCF8574 at the given 7-bit address
 */
esp_err_t pcf8574_read_byte(uint8_t addr, uint8_t *data)
{
	return i2c_master_read_from_device(I2C_PORT, addr, data, 1, pdMS_TO_TICKS(50));
}

/**
 * @brief Drive exactly one of the 16 CS lines low, all others high.
 *
 * Caller (spi_bus.c) is expected to hold i2c_bus_mutex for the entire SPI
 * transaction that follows, not just this call; see i2c_bus.h.
 *
 * @param channel 0-15 (RTD channel n uses channel = n-1)
 */
esp_err_t cs_expander_select(uint8_t channel)
{
	if (channel >= CS_EXPANDER_CHANNEL_COUNT) {
		ESP_LOGE(TAG_I2C, "cs_expander_select: channel %u out of range", channel);
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t new_low  = 0xFF;
	uint8_t new_high = 0xFF;

	if (channel < 8) {
		new_low &= ~(1 << channel);
	} else {
		new_high &= ~(1 << (channel - 8));
	}

	esp_err_t e1 = pcf8574_write_byte(PCF8574_CS_LOW_ADDR, new_low);
	esp_err_t e2 = pcf8574_write_byte(PCF8574_CS_HIGH_ADDR, new_high);

	if (e1 != ESP_OK || e2 != ESP_OK) {
		ESP_LOGE(TAG_I2C, "cs_expander_select(%u) I2C write failed (low=%s, high=%s)",
				 channel, esp_err_to_name(e1), esp_err_to_name(e2));
		// Fail safe: force both expanders back to all-high so we never leave two CS lines
		// asserted at once if one of the two writes above succeeded and the other didn't.
		pcf8574_write_byte(PCF8574_CS_LOW_ADDR, 0xFF);
		pcf8574_write_byte(PCF8574_CS_HIGH_ADDR, 0xFF);
		cs_low_state = 0xFF;
		cs_high_state = 0xFF;
		return ESP_FAIL;
	}

	cs_low_state = new_low;
	cs_high_state = new_high;
	return ESP_OK;
}

/**
 * @brief Return all 16 CS lines to the idle (deselected) state
 */
esp_err_t cs_expander_deselect_all(void)
{
	esp_err_t e1 = pcf8574_write_byte(PCF8574_CS_LOW_ADDR, 0xFF);
	esp_err_t e2 = pcf8574_write_byte(PCF8574_CS_HIGH_ADDR, 0xFF);
	cs_low_state = 0xFF;
	cs_high_state = 0xFF;
	if (e1 != ESP_OK || e2 != ESP_OK) {
		ESP_LOGE(TAG_I2C, "cs_expander_deselect_all failed (low=%s, high=%s)",
				 esp_err_to_name(e1), esp_err_to_name(e2));
		return ESP_FAIL;
	}
	return ESP_OK;
}
