/*
 * spi_bus.h
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 */

#ifndef MAIN_SPI_BUS_H_
#define MAIN_SPI_BUS_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pinout.h"
#include "i2c_bus.h"

/*******************************************************************************************************************************
DEFINITIONS: MAX31865 REGISTERS (Maxim register map, common to every MAX31865 driver)
********************************/
#define MAX31865_CONFIG_REG             0x00
#define MAX31865_CONFIG_BIAS            0x80
#define MAX31865_CONFIG_MODEAUTO        0x40
#define MAX31865_CONFIG_1SHOT           0x20
#define MAX31865_CONFIG_3WIRE           0x10
#define MAX31865_CONFIG_FAULTSTAT       0x02
#define MAX31865_CONFIG_FILT50HZ        0x01

#define MAX31865_RTDMSB_REG             0x01
#define MAX31865_RTDLSB_REG             0x02
#define MAX31865_FAULTSTAT_REG          0x07

#define MAX31865_FAULT_HIGHTHRESH       0x80
#define MAX31865_FAULT_LOWTHRESH        0x40
#define MAX31865_FAULT_REFINLOW         0x20
#define MAX31865_FAULT_REFINHIGH        0x10
#define MAX31865_FAULT_RTDINLOW         0x08
#define MAX31865_FAULT_OVUV             0x04
/******************************************************************************************************************/

/*******************************************************************************************************************
 RTD CONFIGURATION:
 The PCB supports either PT1000 or PT100 sensors. The only hardware difference is the
 reference resistor (RREF) fitted around each MAX31865. This is selected via solder jumpers during assembly

 Change ONLY RTD_SENSOR_TYPE below when building firmware for a different board.

 PT1000 : RREF = 4.3kΩ
 PT100  : RREF = 430Ω
********************************************************************************************************************/
typedef enum
{
    RTD_TYPE_PT1000 = 0,
    RTD_TYPE_PT100  = 1,
} rtd_type_t;

// NOTE: 
// un-comment the correct RTD here. Default is PT1000
#define RTD_SENSOR_TYPE      RTD_TYPE_PT1000
//#define RTD_SENSOR_TYPE      RTD_TYPE_PT100

#if (RTD_SENSOR_TYPE == RTD_TYPE_PT1000)

    #define RTD_NOMINAL_OHMS      1000.0f
    #define RTD_RREF_OHMS         4300.0f

#elif (RTD_SENSOR_TYPE == RTD_TYPE_PT100)

    #define RTD_NOMINAL_OHMS      100.0f
    #define RTD_RREF_OHMS         430.0f

#else
    #error Invalid RTD_SENSOR_TYPE selected.
#endif

#define RTD_CALLENDAR_VANDUSEN_A   3.9083e-3
#define RTD_CALLENDAR_VANDUSEN_B  (-5.775e-7)

// Functional spec: -40 to +200C, 
#define RTD_RANGE_MIN_C           -40.0f
#define RTD_RANGE_MAX_C            200.0f

// Per-sample slew-rate gate for the "unstable reading" validity check
#define RTD_MAX_SLEW_C_PER_SCAN     5.0f
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
CHANNEL FAULT / VALIDITY FLAGS 
********************************/
#define RTD_FAULT_NONE            0x00
#define RTD_FAULT_OPEN_CIRCUIT    0x01   // MAX31865 RTDIN/REFIN fault bits
#define RTD_FAULT_OUT_OF_RANGE    0x02   // Converted temperature outside RTD_RANGE_MIN_C..MAX_C
#define RTD_FAULT_UNSTABLE        0x04   // Slew-rate gate tripped
#define RTD_FAULT_UNCALIBRATED    0x08   // NVS coefficients missing/corrupt for this channel, reading is raw, not trusted
/*******************************************************************************************************************************/

typedef struct {
	float offset_c;     
	float gain;      
	bool  loaded;       
} rtd_calibration_t;

typedef struct {
	float   temperature_c[RTD_CHANNEL_COUNT];  
	uint8_t fault[RTD_CHANNEL_COUNT];          
	bool    valid[RTD_CHANNEL_COUNT];           
	uint32_t sequence;                        
} rtd_scan_result_t;

esp_err_t spi_bus_init(void);
esp_err_t rtd_calibration_load_all(void);
esp_err_t rtd_calibration_set(uint8_t channel, float offset_c, float gain);

// Performs one full 16-channel round-robin scan and fills out.
esp_err_t rtd_scan_all_channels(rtd_scan_result_t *out);

#endif /* MAIN_SPI_BUS_H_ */
