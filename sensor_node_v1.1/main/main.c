/***********************************************************************************************************************
 ∎ PROJECT:
        OCTAVIA CARBON -- SENSOR NODE (ON-SITE POC BUILD)
 ∎ BRIEF:
        Proof-of-concept pipeline test on Octavia Carbon's own hardware. Reads 16x
        3-wire RTDs (MAX31865 front ends) over a shared SPI bus, chip select via a
        single MCP23017 I2C GPIO expander. Modbus RTU SLAVE on RS485 (the S7-1200
        PLC is the master), broadcast-only unacknowledged publisher on CAN (the
        Embedded Linux IPC listens). No calibration, no local UI -- this build's
        only job is proving the acquisition -> Modbus and acquisition -> CAN paths
        work end to end on real hardware.

 ∎ ARCHITECTURE:
        Three tasks, same decoupling as the full design:

        • Acquisition Task  -> Core 1  (SPI/I2C: 16-channel RTD scan)
        • Modbus Task       -> Core 0  (RS485: services PLC polls, never blocked by CAN or SPI)
        • CAN Task          -> Core 0  (broadcasts each completed scan, decoupled from Modbus)

        The Acquisition Task is the only writer of RTD data. It publishes into the
        Modbus input-register table (mutex-protected) and hands the CAN task a copy
        via a length-1 overwrite queue. Modbus and CAN tasks never touch the SPI or
        I2C bus directly and never wait on each other.

 ∎ FIRMWARE: ERIC MULWA
 ∎ CONTACT:
        Email : mulwaericbsc@gmail.com
        Phone : +254 796 456 877
 ∎ DATES:
        Created : 03/08/2026
 ∎ SUBMITTED FOR:
        Octavia Carbon: Instrumentation & Control Engineer -- on-site implementation

************************************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_system.h"
#include "esp_log.h"
#include "pinout.h"
#include "i2c_bus.h"
#include "spi_bus.h"
#include "modbus.h"
#include "canbus.h"

/***********************************************************************************************************************/
static const char *TAG = "SENSOR_NODE";
/***********************************************************************************************************************/

/************************************************************************************************************************
Acquisition cadence: one 16-channel scan takes 1.1-1.3s (16 x 75ms bias+convert), padded to
a round 2s cycle so scan timing is predictable for latency budget.
********************************/
#define ACQUISITION_CYCLE_MS    2000
static rtd_scan_result_t latest_scan;
static bool spi_last_ok = false;
/***********************************************************************************************************************/

/************************************************************************************************************************
FUNCTIONS: PROTOTYPES
**********************/
void printAsteriskSeparator(int length);
void print_scan_diagnostics(const rtd_scan_result_t *scan);

void acquisition_task(void *arg);
void modbus_task(void *arg);
void can_task(void *arg);
/**********************************************************************************************************************/


/************************************************************************************************************************
FUNCTIONS: MISC
****************/
void printAsteriskSeparator(int length)
{
	char separator[length + 1];
	for (int i = 0; i < length; i++) {
		separator[i] = '*';
	}
	separator[length] = '\0';
	printf("\n%s\n", separator);
}

void print_scan_diagnostics(const rtd_scan_result_t *scan)
{
	printf("Scan sequence #%lu\n", (unsigned long)scan->sequence);
	for (int ch = 0; ch < RTD_CHANNEL_COUNT; ch++) {
		printf("  CH%02d : %7.2f C   valid=%d   fault=0x%02X\n",
			   ch + 1, scan->temperature_c[ch], scan->valid[ch], scan->fault[ch]);
	}
}
/***********************************************************************************************************************/


/************************************************************************************************************************
RTOS TASK: Acquisition: Core 1
*******************************/
void acquisition_task(void *arg)
{
	for (;;) {
		TickType_t cycle_start = xTaskGetTickCount();

		esp_err_t err = rtd_scan_all_channels(&latest_scan);
		spi_last_ok = (err == ESP_OK);

		modbus_slave_update_data(&latest_scan, spi_last_ok, true);
		canbus_notify_new_scan(&latest_scan);

		// On-site console visibility -- printed every cycle so readings are visible on the
		// serial monitor without needing a Modbus/CAN sniffer to confirm the pipeline works.
		print_scan_diagnostics(&latest_scan);

		if (!spi_last_ok) {
			ESP_LOGW(TAG, "Scan failed this cycle");
		}

		TickType_t elapsed = xTaskGetTickCount() - cycle_start;
		TickType_t target = pdMS_TO_TICKS(ACQUISITION_CYCLE_MS);
		if (elapsed < target) {
			vTaskDelay(target - elapsed);
		}
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}
/***********************************************************************************************************************/


/************************************************************************************************************************
RTOS TASK: Modbus - Core 0. Tight loop; modbus_slave_poll() blocks on UART with a
200ms window, so this never busy-spins and never depends on the acquisition or CAN task.
*******************************/
void modbus_task(void *arg)
{
	for (;;) {
		modbus_slave_poll();
	}
}
/************************************************************************************************************************/


/************************************************************************************************************************
RTOS TASK: CAN - Core 0. Blocks on the acquisition task's scan-ready queue.
*******************************/
void can_task(void *arg)
{
	for (;;) {
		canbus_publish_task_body();
	}
}
/***********************************************************************************************************************/


/************************************************************************************************************************
FUNCTION: MAIN APPLICATION
**************************/
void app_main(void)
{
	printAsteriskSeparator(100);
	printf("OCTAVIA CARBON SENSOR NODE (POC BUILD): booting...\n");
	printAsteriskSeparator(100);
	vTaskDelay(pdMS_TO_TICKS(100));

	if (i2c_bus_init() != ESP_OK) {
		ESP_LOGE(TAG, "I2C initialization failed!");
		esp_restart();
	} else {
		ESP_LOGI(TAG, "I2C initialized successfully!");
	}
	vTaskDelay(pdMS_TO_TICKS(100));

	if (spi_bus_init() != ESP_OK) {
		ESP_LOGE(TAG, "SPI initialization failed!");
		esp_restart();
	} else {
		ESP_LOGI(TAG, "SPI initialized successfully!");
	}
	
	vTaskDelay(pdMS_TO_TICKS(100));

	if (modbus_slave_init() != ESP_OK) {
		ESP_LOGE(TAG, "Modbus initialization failed!");
		esp_restart();
	} else {
		ESP_LOGI(TAG, "Modbus initialized successfully!");
	}
	vTaskDelay(pdMS_TO_TICKS(100));

	if (canbus_init() != ESP_OK) {
		ESP_LOGE(TAG, "CAN initialization failed!");
		esp_restart();
	} else {
		ESP_LOGI(TAG, "CAN initialized successfully!");
	}
	vTaskDelay(pdMS_TO_TICKS(100));

	printf("Sensor Node initialized successfully!\n");
	vTaskDelay(pdMS_TO_TICKS(100));

	memset(&latest_scan, 0, sizeof(latest_scan));

	xTaskCreatePinnedToCore(acquisition_task, "acquisition_task", 4096, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(modbus_task, "modbus_task", 4096, NULL, 4, NULL, 0);
	xTaskCreatePinnedToCore(can_task, "can_task", 4096, NULL, 2, NULL, 0);
}
/***********************************************************************************************************************/
