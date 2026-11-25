/*
 * spi_bus.c
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 */
#include "spi_bus.h"

static const char *TAG_SPI = "SPI_BUS";

static spi_device_handle_t rtd_spi_handle;
static float last_good_temp_c[RTD_CHANNEL_COUNT];
static bool  have_last_good[RTD_CHANNEL_COUNT];

/**
 * @brief Initialize the shared SPI bus and attach the single logical MAX31865 device.
 */
esp_err_t spi_bus_init(void)
{
	printf("Initializing the SPI master...\n");

	spi_bus_config_t bus_cfg = {
		.mosi_io_num = SPI_MOSI_PIN,
		.miso_io_num = SPI_MISO_PIN,
		.sclk_io_num = SPI_SCLK_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4,
	};

	esp_err_t err = spi_bus_initialize(SPI_HOST_BUS, &bus_cfg, SPI_DMA_DISABLED);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG_SPI, "spi_bus_initialize failed: %s", esp_err_to_name(err));
		return err;
	}

	spi_device_interface_config_t dev_cfg = {
		.clock_speed_hz = SPI_CLOCK_HZ,
		.mode = 1,                  // MAX31865: CPOL=0, CPHA=1
		.spics_io_num = -1,         // No native CS - handled externally via cs_expander_select()
		.queue_size = 1,
		//.flags = SPI_DEVICE_HALFDUPLEX,
	};

	err = spi_bus_add_device(SPI_HOST_BUS, &dev_cfg, &rtd_spi_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG_SPI, "spi_bus_add_device failed: %s", esp_err_to_name(err));
		return err;
	}

	for (int ch = 0; ch < RTD_CHANNEL_COUNT; ch++) {
		have_last_good[ch] = false;
	}

	ESP_LOGI(TAG_SPI, "SPI bus ready, 16 MAX31865 channels addressable via CS-expander");
	return ESP_OK;
}

static esp_err_t max31865_write_register(uint8_t addr, uint8_t data)
{
	uint8_t tx[2] = { (uint8_t)(addr | 0x80), data };
	spi_transaction_t t = {
		.length = 16,
		.tx_buffer = tx,
	};
	return spi_device_transmit(rtd_spi_handle, &t);
}

static esp_err_t max31865_read_register(uint8_t addr, uint8_t *out)
{
	uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
	uint8_t rx[2] = { 0 };
	spi_transaction_t t = {
		.length = 16,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};
	esp_err_t err = spi_device_transmit(rtd_spi_handle, &t);
	if (err != ESP_OK) return err;
	*out = rx[1];
	return ESP_OK;
}

static esp_err_t max31865_read_rtd_raw(uint16_t *raw_out)
{
	uint8_t tx[3] = { MAX31865_RTDMSB_REG & 0x7F, 0x00, 0x00 };
	uint8_t rx[3] = { 0 };
	spi_transaction_t t = {
		.length = 24,
		.tx_buffer = tx,
		.rx_buffer = rx,
	};
	esp_err_t err = spi_device_transmit(rtd_spi_handle, &t);
	if (err != ESP_OK) return err;
	*raw_out = (((uint16_t)rx[1] << 8) | rx[2]) >> 1;   // 15-bit RTD ADC code, fault bit stripped
	return ESP_OK;
}

/**
 * @brief Configure one MAX31865 channel: bias off, auto-convert off, 3-wire, 50Hz filter,
 *        clear faults.
 */
static esp_err_t max31865_configure_channel(uint8_t channel)
{
	esp_err_t err = cs_expander_select(channel);
	if (err != ESP_OK) return err;

	uint8_t cfg = MAX31865_CONFIG_3WIRE | MAX31865_CONFIG_FILT50HZ;
	err = max31865_write_register(MAX31865_CONFIG_REG, cfg);

	cs_expander_deselect_all();
	return err;
}

/**
 * @brief One-shot RTD read on a single channel: enable bias, settle, trigger conversion, wait,
 *        read RTD code and fault register.
 */
static esp_err_t max31865_read_channel(uint8_t channel, uint16_t *raw_rtd, uint8_t *fault_reg)
{
	esp_err_t err = cs_expander_select(channel);
	if (err != ESP_OK) return err;

	uint8_t cfg = MAX31865_CONFIG_3WIRE | MAX31865_CONFIG_FILT50HZ | MAX31865_CONFIG_BIAS;
	max31865_write_register(MAX31865_CONFIG_REG, cfg);
	vTaskDelay(pdMS_TO_TICKS(10));      // Bias settle, per Maxim datasheet timing

	cfg |= MAX31865_CONFIG_1SHOT;
	max31865_write_register(MAX31865_CONFIG_REG, cfg);
	vTaskDelay(pdMS_TO_TICKS(65));      // 1-shot conversion time (50Hz filter - 62.5ms worst case)
	
// --- DIAGNOSTIC: confirm the write actually landed on the chip --- TODO
uint8_t cfg_readback = 0;
max31865_read_register(MAX31865_CONFIG_REG, &cfg_readback);
ESP_LOGI(TAG_SPI, "CH%d config readback: 0x%02X (wrote 0x%02X)", channel + 1, cfg_readback, cfg);
// ------------------------------------------------------------------

	err = max31865_read_rtd_raw(raw_rtd);
	if (err == ESP_OK) {
		err = max31865_read_register(MAX31865_FAULTSTAT_REG, fault_reg);
	}

	// Clear fault latch + turn bias back off between channels
	uint8_t clear_cfg = MAX31865_CONFIG_3WIRE | MAX31865_CONFIG_FILT50HZ | MAX31865_CONFIG_FAULTSTAT;
	max31865_write_register(MAX31865_CONFIG_REG, clear_cfg);

	cs_expander_deselect_all();
	return err;
}

/**
 * @brief Callendar-Van Dusen conversion, RTD resistance -> temperature (deg C).
 */
static float rtd_resistance_to_celsius(float rtd_ohms, float rtd_nominal_ohms)
{
	float Z1, Z2, Z3, Z4, temp;

	Z1 = -RTD_CALLENDAR_VANDUSEN_A;
	Z2 = RTD_CALLENDAR_VANDUSEN_A * RTD_CALLENDAR_VANDUSEN_A - (4 * RTD_CALLENDAR_VANDUSEN_B);
	Z3 = (4 * RTD_CALLENDAR_VANDUSEN_B) / rtd_nominal_ohms;
	Z4 = 2 * RTD_CALLENDAR_VANDUSEN_B;

	temp = Z2 + (Z3 * rtd_ohms);
	temp = (sqrtf(temp) + Z1) / Z4;

	if (temp >= 0.0f) {
		return temp;
	}

	// Sub-zero polynomial correction (Maxim AN709)
	float Rt = rtd_ohms / rtd_nominal_ohms * 100.0f;
	float rpoly = Rt;
	temp = -242.02f;
	temp += 2.2228f * rpoly;
	rpoly *= Rt;
	temp += 2.5859e-3f * rpoly;
	rpoly *= Rt;
	temp -= 4.8260e-6f * rpoly;
	rpoly *= Rt;
	temp -= 2.8183e-8f * rpoly;
	rpoly *= Rt;
	temp += 1.5243e-10f * rpoly;
	return temp;
}

/**
 * @brief One full round-robin scan of all 16 RTD channels. No calibration applied in this
 *        POC build -- temperature_c is the raw Callendar-Van Dusen conversion.
 */
/*esp_err_t rtd_scan_all_channels(rtd_scan_result_t *out)
{
	static uint32_t scan_sequence = 0;

	for (uint8_t ch = 0; ch < RTD_CHANNEL_COUNT; ch++) {

		if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
			ESP_LOGE(TAG_SPI, "Channel %d: could not acquire CS-expander mutex, marking fault", ch);
			out->fault[ch] |= RTD_FAULT_OPEN_CIRCUIT;
			out->valid[ch] = false;
			continue;
		}

		max31865_configure_channel(ch);

		uint16_t raw_rtd = 0;
		uint8_t fault_reg = 0;
		esp_err_t err = max31865_read_channel(ch, &raw_rtd, &fault_reg);

		xSemaphoreGive(i2c_bus_mutex);

		uint8_t flags = RTD_FAULT_NONE;

		if (err != ESP_OK) {
			flags |= RTD_FAULT_OPEN_CIRCUIT;
			out->fault[ch] = flags;
			out->valid[ch] = false;
			continue;
		}

		if (fault_reg & (MAX31865_FAULT_RTDINLOW | MAX31865_FAULT_REFINLOW | MAX31865_FAULT_REFINHIGH)) {
			flags |= RTD_FAULT_OPEN_CIRCUIT;
		}
		if (fault_reg & (MAX31865_FAULT_HIGHTHRESH | MAX31865_FAULT_LOWTHRESH | MAX31865_FAULT_OVUV)) {
			flags |= RTD_FAULT_OUT_OF_RANGE;
		}

		float rtd_ohms = ((float)raw_rtd / 32768.0f) * RTD_RREF_OHMS;
		float temp_c = rtd_resistance_to_celsius(rtd_ohms, RTD_NOMINAL_OHMS);

		if (temp_c < RTD_RANGE_MIN_C || temp_c > RTD_RANGE_MAX_C) {
			flags |= RTD_FAULT_OUT_OF_RANGE;
		}

		if (have_last_good[ch] && fabsf(temp_c - last_good_temp_c[ch]) > RTD_MAX_SLEW_C_PER_SCAN) {
			flags |= RTD_FAULT_UNSTABLE;
		}

		out->temperature_c[ch] = temp_c;
		out->fault[ch] = flags;
		out->valid[ch] = !(flags & (RTD_FAULT_OPEN_CIRCUIT | RTD_FAULT_OUT_OF_RANGE | RTD_FAULT_UNSTABLE));

		if (out->valid[ch]) {
			last_good_temp_c[ch] = temp_c;
			have_last_good[ch] = true;
		}
	}

	out->sequence = ++scan_sequence;
	return ESP_OK;
}*/

esp_err_t rtd_scan_all_channels(rtd_scan_result_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return rtd_scan_hardcoded(out);
}

/**
 * @brief POC/test mode: populate all RTD channels with fixed temperatures.
 *
 * This bypasses the MAX31865/SPI acquisition completely.
 * The values are presented to the rest of the application exactly
 * like valid RTD measurements.
 */
esp_err_t rtd_scan_hardcoded(rtd_scan_result_t *out)
{
    static const float hardcoded_temperatures[RTD_CHANNEL_COUNT] = {
        25.0f,   // CH01
        26.0f,   // CH02
        27.0f,   // CH03
        28.0f,   // CH04
        29.0f,   // CH05
        30.0f,   // CH06
        31.0f,   // CH07
        32.0f,   // CH08
        33.0f,   // CH09
        34.0f,   // CH10
        35.0f,   // CH11
        36.0f,   // CH12
        37.0f,   // CH13
        38.0f,   // CH14
        39.0f,   // CH15
        40.0f    // CH16
    };

    static uint32_t scan_sequence = 0;

    for (uint8_t ch = 0; ch < RTD_CHANNEL_COUNT; ch++) {

        out->temperature_c[ch] = hardcoded_temperatures[ch];

        // No simulated fault
        out->fault[ch] = RTD_FAULT_NONE;

        // Treat the hardcoded values as valid RTD readings
        out->valid[ch] = true;
    }

    out->sequence = ++scan_sequence;

    return ESP_OK;
}
