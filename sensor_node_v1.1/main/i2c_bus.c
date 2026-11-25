/*
 * i2c_bus.c
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 */
#include "i2c_bus.h"

static const char *TAG_I2C = "I2C_BUS";

SemaphoreHandle_t i2c_bus_mutex = NULL;

/*******************************************************************************************************************************
MCP23017 register map (IOCON.BANK = 0, the power-on default -- Port A and Port B registers
are interleaved by address, e.g. IODIRA=0x00/IODIRB=0x01, GPIOA=0x12/GPIOB=0x13, etc.)
********************************/
#define MCP23017_REG_IODIRA     0x00   // 1 = input, 0 = output
#define MCP23017_REG_IODIRB     0x01
#define MCP23017_REG_GPPUA      0x0C   // Pull-up enable (unused here -- all pins are outputs)
#define MCP23017_REG_GPPUB      0x0D
#define MCP23017_REG_GPIOA      0x12
#define MCP23017_REG_GPIOB      0x13
#define MCP23017_REG_OLATA      0x14   // Output latch -- what we actually write CS states to
#define MCP23017_REG_OLATB      0x15
/*******************************************************************************************************************************/

// Shadow state, CS1-CS8 on Port A, CS9-CS16 on Port B. All-1 = all 16 CS lines deselected (high).
static uint8_t cs_porta_state = 0xFF;
static uint8_t cs_portb_state = 0xFF;

static esp_err_t mcp23017_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = { reg, value };
	return i2c_master_write_to_device(I2C_PORT, MCP23017_ADDR, buf, 2, pdMS_TO_TICKS(50));
}

/**
 * @brief Initialize the I2C master bus and the MCP23017 CS-expander
 */
esp_err_t i2c_bus_init(void)
{
	printf("Initializing the I2C master (CS-expander bus)...\n");

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
	printf("i2c_param_configured.\n");
	
	err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_I2C, "i2c_driver_install failed: %s", esp_err_to_name(err));
		return err;
	}
	printf("i2c_driver_installed.\n");

	i2c_bus_mutex = xSemaphoreCreateMutex();
	if (i2c_bus_mutex == NULL) {
		ESP_LOGE(TAG_I2C, "Failed to create I2C bus mutex");
		return ESP_FAIL;
	}
	printf("created I2C bus mutex.\n");
	
	i2c_scan();
	vTaskDelay(pdMS_TO_TICKS(10));

	// Both ports: all 16 pins outputs, no pull-ups needed, idle state = all high (CS deselected)
	esp_err_t e1 = mcp23017_write_reg(MCP23017_REG_IODIRA, 0x00);
	esp_err_t e2 = mcp23017_write_reg(MCP23017_REG_IODIRB, 0x00);
	esp_err_t e3 = mcp23017_write_reg(MCP23017_REG_GPPUA, 0x00);
	esp_err_t e4 = mcp23017_write_reg(MCP23017_REG_GPPUB, 0x00);
	esp_err_t e5 = mcp23017_write_reg(MCP23017_REG_OLATA, 0xFF);
	esp_err_t e6 = mcp23017_write_reg(MCP23017_REG_OLATB, 0xFF);

	if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK || e4 != ESP_OK || e5 != ESP_OK || e6 != ESP_OK) {
		ESP_LOGE(TAG_I2C, "MCP23017 init failed (dir=%s/%s pu=%s/%s olat=%s/%s)",
				 esp_err_to_name(e1), esp_err_to_name(e2), esp_err_to_name(e3),
				 esp_err_to_name(e4), esp_err_to_name(e5), esp_err_to_name(e6));
		return ESP_FAIL;
	}

	cs_porta_state = 0xFF;
	cs_portb_state = 0xFF;

	ESP_LOGI(TAG_I2C, "I2C bus + MCP23017 ready, all 16 channels deselected");
	return ESP_OK;
}

void i2c_scan()
{
    printf("\nI2C scan starting...\n");

    for (uint8_t addr = 1; addr < 127; addr++) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);

        i2c_master_write_byte(
            cmd,
            (addr << 1) | I2C_MASTER_WRITE,
            true
        );

        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(
            I2C_PORT,
            cmd,
            pdMS_TO_TICKS(100)
        );

        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("I2C device found at 0x%02X\n", addr);
        }
    }

    printf("I2C scan complete.\n");
}

/**
 * @brief Drive exactly one of the 16 CS lines low, all others high.
 *
 * Caller (spi_bus.c) is expected to hold i2c_bus_mutex for the entire SPI
 * transaction that follows, not just this call -- see i2c_bus.h.
 *
 * @param channel 0-15 (RTD channel n uses channel = n-1; CS1..CS8 = Port A, CS9..CS16 = Port B)
 */
esp_err_t cs_expander_select(uint8_t channel)
{
	if (channel >= CS_EXPANDER_CHANNEL_COUNT) {
		ESP_LOGE(TAG_I2C, "cs_expander_select: channel %u out of range", channel);
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t new_a = 0xFF;
	uint8_t new_b = 0xFF;

	/*if (channel < 8) {
		new_a &= ~(1 << channel);
	} else {
		new_b &= ~(1 << (channel - 8));
	}*/

if (channel < 8) {
    // Port A confirmed ascending: GPA0=CS1 ... GPA7=CS8
    new_a &= ~(1 << channel);
} else {
    // Port B confirmed REVERSED on this board: GPB0=CS16 ... GPB7=CS9 (not ascending
    // like Port A). channel 8 = CS9 -> bit 7, channel 15 = CS16 -> bit 0.
    uint8_t portb_bit = 15 - channel;
    new_b &= ~(1 << portb_bit);
}

	esp_err_t e1 = mcp23017_write_reg(MCP23017_REG_OLATA, new_a);
	esp_err_t e2 = mcp23017_write_reg(MCP23017_REG_OLATB, new_b);

	if (e1 != ESP_OK || e2 != ESP_OK) {
		ESP_LOGE(TAG_I2C, "cs_expander_select(%u) I2C write failed (A=%s, B=%s)",
				 channel, esp_err_to_name(e1), esp_err_to_name(e2));
		// Fail safe: force both ports back to all-high so we never leave two CS lines
		// asserted at once if one of the two writes above succeeded and the other didn't.
		mcp23017_write_reg(MCP23017_REG_OLATA, 0xFF);
		mcp23017_write_reg(MCP23017_REG_OLATB, 0xFF);
		cs_porta_state = 0xFF;
		cs_portb_state = 0xFF;
		return ESP_FAIL;
	}

	cs_porta_state = new_a;
	cs_portb_state = new_b;
	return ESP_OK;
}

/**
 * @brief Return all 16 CS lines to the idle (deselected) state
 */
esp_err_t cs_expander_deselect_all(void)
{
	esp_err_t e1 = mcp23017_write_reg(MCP23017_REG_OLATA, 0xFF);
	esp_err_t e2 = mcp23017_write_reg(MCP23017_REG_OLATB, 0xFF);
	cs_porta_state = 0xFF;
	cs_portb_state = 0xFF;
	if (e1 != ESP_OK || e2 != ESP_OK) {
		ESP_LOGE(TAG_I2C, "cs_expander_deselect_all failed (A=%s, B=%s)",
				 esp_err_to_name(e1), esp_err_to_name(e2));
		return ESP_FAIL;
	}
	return ESP_OK;
}
