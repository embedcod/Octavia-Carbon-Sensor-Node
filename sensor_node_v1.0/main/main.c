/**********************************************************************************************************************
 ∎ PROJECT:
        OCTAVIA CARBON -- SENSOR NODE
 ∎ BRIEF:
        Firmware for the custom Sensor Node PCB submitted for the Octavia Carbon
        Instrumentation & Control Engineer Design Challenge. The node reads 16
        3-wire PT1000 RTDs (MAX31865 front ends) over a shared SPI bus, with chip
        select fanned out through two I2C PCF8574 GPIO expanders. It is a Modbus
        RTU SLAVE on RS485 (the Siemens S7-1200 PLC is the master and sole owner
        of the valve interlock) and a broadcast-only, unacknowledged publisher on
        CAN (the Embedded Linux IPC listens, the PLC is not on this bus).

 ∎ ARCHITECTURE:
        Deterministic FreeRTOS Safe-Thread system, tasks pinned for isolation and
        predictability:

        • Acquisition Task  → Core 1  (SPI/I2C: 16-channel RTD scan, owns measurement + calibration)
        • Modbus Task       → Core 0  (RS485: services PLC polls, never blocked by CAN or SPI)
        • CAN Task          → Core 0  (broadcasts each completed scan, decoupled from Modbus)
        • Button Task       → unpinned (local service button: debounce, multi-press, diagnostics)

        The Acquisition Task is the only writer of RTD data. It publishes into the
        Modbus input-register table (mutex-protected) and hands the CAN task a copy
        via a length-1 overwrite queue. Modbus and CAN tasks never touch the SPI or
        I2C bus directly and never wait on each other. This is what guarantees a
        slow ADC conversion or a stalled CAN frame cannot delay a PLC poll response.

 ∎ FEATURES:
        • 16-channel RTD acquisition with per-channel calibration (NVS-backed)
        • Per-channel fault detection: open circuit, out-of-range, unstable, uncalibrated
        • Modbus RTU slave (input + holding registers, calibration write-through)
        • CAN broadcast publisher (multiplexed 5-frame burst per scan)
        • Button interrupt handling with debounce and multi-press logic (local diagnostics only)
        • Status LED heartbeat + fault indication

 ∎ FIRMWARE: DESIGN:
        • RTOS-driven, non-blocking architecture
        • Interrupt-driven user input handling
        • Fail-safe: any bus or read failure degrades a channel's validity flag, never fabricates data

 ∎ FIRMWARE: ERIC MULWA
 ∎ HARDWARE: ERIC MULWA
 ∎ CONTACT:
        Email : mulwaericbsc@gmail.com
        Phone : +254 796 456 877
 ∎ DATES:
        Created : 03/08/2026
 ∎ SUBMITTED FOR:
        Octavia Carbon: Instrumentation & Control Engineer Design Challenge

************************************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pinout.h"
#include "i2c_bus.h"
#include "spi_bus.h"
#include "modbus.h"
#include "canbus.h"

/***********************************************************************************************************************/
static const char *TAG = "SENSOR_NODE";

/***********************************************************************************************************************/

/************************************************************************************************************************
Button Definitions
********************************/
#define DEBOUNCE_TIME_MS        300
#define MULTI_PRESS_DELAY_MS    2000
#define MIN_PRESS_INTERVAL_MS   300

volatile int  buttonPressCount = 0;
volatile bool interruptFlag = false;
TimerHandle_t pressTimer;
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
void setup_status_led(void);
void heartbeat_led(void);
void fault_led(int count);
void setupButton(void);
void initializeTimer(void);
void handleButtonPress(void);
void print_scan_diagnostics(const rtd_scan_result_t *scan);
void resetDevice(void);
void powerdown();

void acquisition_task(void *arg);
void modbus_task(void *arg);
void can_task(void *arg);
void button_task(void *arg);
/**********************************************************************************************************************/


/***********************************************************************************************************************
FUNCTION: BUTTON INTERRUPT HANDLER
*********************************/
static void IRAM_ATTR button_isr_handler(void *arg)
{
	static unsigned long lastInterruptTime = 0;
	static unsigned long lastPressTime = 0;
	unsigned long interruptTime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

	if (interruptTime - lastInterruptTime > DEBOUNCE_TIME_MS) {
		if (interruptTime - lastPressTime > MIN_PRESS_INTERVAL_MS) {
			buttonPressCount++;
			xTimerResetFromISR(pressTimer, NULL);
			lastPressTime = interruptTime;
			interruptFlag = true;
		}
	}
	lastInterruptTime = interruptTime;
}

/**
 * @brief Multi-press dispatch. The button is diagnostics/service only: 
 *		  single press dumps the current scan to the
 *        console, a double press does a clean restart.
 */
void handleButtonPress(void)
{
	if (!interruptFlag) return;

	switch (buttonPressCount) {
		case 1:
			printAsteriskSeparator(100);
			printf("Service button: single press, dumping current scan snapshot\n");
			print_scan_diagnostics(&latest_scan);
			break;
		case 2:
			printAsteriskSeparator(100);
			printf("Service button: double press, restarting Sensor Node\n");
			fault_led(4);
			resetDevice();
			break;
		default:
			printAsteriskSeparator(100);
			printf("Service button: shutting down Sensor Node...\n");
			fault_led(6);
			powerdown();
			break;
	}

	interruptFlag = false;
	buttonPressCount = 0;
}

void pressTimerCallback(TimerHandle_t xTimer)
{
	interruptFlag = true;
}

void initializeTimer(void)
{
	pressTimer = xTimerCreate("PressTimer", pdMS_TO_TICKS(MULTI_PRESS_DELAY_MS), pdFALSE, NULL, pressTimerCallback);
}

void setupButton(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;   
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ONLY;
    gpio_config(&io_conf);
    // Install ISR service and add button interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
}
/*********************************************************************************************************************/


/**********************************************************************************************************************
FUNCTIONS: STATUS LED
**********************/
void setup_status_led(void)
{
	gpio_config_t io_conf = {};
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << STATUS_LED_PIN);
	gpio_config(&io_conf);
	gpio_set_level(STATUS_LED_PIN, 0);
}

/**
 * @brief One short blink per completed scan: a healthy node blinks once every ~2s.
 */
void heartbeat_led(void)
{
	gpio_set_level(STATUS_LED_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(100));
	gpio_set_level(STATUS_LED_PIN, 0);
}

/**
 * @brief Fast blink burst: bus/scan fault or a user-triggered action
 */
void fault_led(int count)
{
	for (int i = 0; i < count; i++) {
		gpio_set_level(STATUS_LED_PIN, 1);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(STATUS_LED_PIN, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
/***********************************************************************************************************************/


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
		printf("  CH%02d : %7.2f C   valid=%d   fault=0x%02X%s\n",
			   ch + 1, scan->temperature_c[ch], scan->valid[ch], scan->fault[ch],
			   (ch == 0) ? "   <-- control channel" : "");
	}
}

/**
 * @brief Reset
 */
void resetDevice(void)
{
	printAsteriskSeparator(100);
	printf("Restarting Sensor Node...\n");
	vTaskDelay(pdMS_TO_TICKS(1000));
	esp_restart();
}

/**
 * @brief Power Off
 */
void powerdown() {
	    esp_rom_gpio_pad_select_gpio(POWER_PIN);
		gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
		gpio_set_level(POWER_PIN, 1);	
	    vTaskDelay(pdMS_TO_TICKS(1000)); 
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

		modbus_slave_update_data(&latest_scan, spi_last_ok, true, canbus_last_publish_ok);
		vTaskDelay(pdMS_TO_TICKS(100));
		canbus_notify_new_scan(&latest_scan);

		if (spi_last_ok) {
			heartbeat_led();
		} else {
			ESP_LOGW(TAG, "Scan failed this cycle: flagging fault on LED");
			fault_led(2);
		}

		TickType_t elapsed = xTaskGetTickCount() - cycle_start;
		TickType_t target = pdMS_TO_TICKS(ACQUISITION_CYCLE_MS);
		if (elapsed < target) {
			vTaskDelay(target - elapsed);
		}
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
RTOS TASK: Button - unpinned, low priority. 
*******************************/
void button_task(void *arg)
{
	for (;;) {
		if (interruptFlag) {
			handleButtonPress();
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
/***********************************************************************************************************************/


/************************************************************************************************************************
FUNCTION: MAIN APPLICATION
**************************/
void app_main(void)
{
	printAsteriskSeparator(100);
	printf("OCTAVIA CARBON SENSOR NODE: booting...\n");
	printAsteriskSeparator(100);

	esp_err_t nvs_err = nvs_flash_init();
	if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		nvs_err = nvs_flash_init();
	}
    if (nvs_err != ESP_OK) {
       ESP_LOGE(TAG, "nvs flash initialization failed!");
       esp_restart();
    } else {
        ESP_LOGI(TAG, "nvs flash initialized successfully!");
    }
	
	setup_status_led();
	setupButton();
	initializeTimer();
	// Initialize i2c bus
    if (i2c_bus_init() != ESP_OK) {
       ESP_LOGE(TAG, "I2C initialization failed!");
       esp_restart();
    } else {
        ESP_LOGI(TAG, "I2C initialized successfully!");
    }	
	// initialize spi bus
    if (spi_bus_init() != ESP_OK) {
       ESP_LOGE(TAG, "SPI initialization failed!");
       esp_restart();
    } else {
        ESP_LOGI(TAG, "SPI initialized successfully!");
    }
	// load calibration data
    if (rtd_calibration_load_all() != ESP_OK) {
       ESP_LOGE(TAG, "calibration data load failed!");
    } else {
        ESP_LOGI(TAG, "calibration data loaded successfully!");
    }
	// initialize modbus
    if (modbus_slave_init() != ESP_OK) {
       ESP_LOGE(TAG, "modbus initialization failed!");
       esp_restart();
    } else {
        ESP_LOGI(TAG, "modbus initialized successfully!");
    }
	// initialize can bus
    if (canbus_init() != ESP_OK) {
       ESP_LOGE(TAG, "modbus initialization failed!");
       esp_restart();
    } else {
        ESP_LOGI(TAG, "modbus initialized successfully!");
    }

	printf("Sensor Node Initialized successfully!\n");

	memset(&latest_scan, 0, sizeof(latest_scan));
	
	// Tasks
	xTaskCreatePinnedToCore(acquisition_task, "acquisition_task", 4096, NULL, 3, NULL, 1);
	xTaskCreatePinnedToCore(modbus_task, "modbus_task", 4096, NULL, 4, NULL, 0);
	xTaskCreatePinnedToCore(can_task, "can_task", 4096, NULL, 2, NULL, 0);
	xTaskCreate(button_task, "button_task", 2048, NULL, 1, NULL);

}
/***********************************************************************************************************************/
