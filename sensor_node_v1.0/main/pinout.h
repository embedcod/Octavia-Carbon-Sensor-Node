/*
 * pinout.h
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 */

#ifndef MAIN_PINOUT_H_
#define MAIN_PINOUT_H_

#include "driver/gpio.h"
#include "driver/uart.h"

/***********************************************************************************************************************
I2C BUS: shared bus to the two PCF8574 CS-expanders (U8 @ 0x20, U9 @ 0x27)
*************************************************************************/
#define I2C_SDA_PIN             GPIO_NUM_21    
#define I2C_SCL_PIN             GPIO_NUM_22  
#define I2C_PORT                I2C_NUM_0
#define I2C_CLK_HZ              400000          // 400kHz: fast enough for CS toggling between SPI words

// PCF8574 CS-expander addresses (7-bit)
//   U8: "Read Address: 41  Write Address: 40"  7-bit 0x20 -> drives CS1..CS8  (P0..P7)
//   U9: "Read Address: 4F  Write Address: 4E"  7-bit 0x27 -> drives CS9..CS16 (P0..P7)
#define PCF8574_CS_LOW_ADDR     0x20            // CS1-CS8
#define PCF8574_CS_HIGH_ADDR    0x27            // CS9-CS16

/************************************************************************************************************************
SPI BUS: shared bus to all 16 MAX31865 RTD Amplifiers
****************************************************/
#define SPI_HOST_BUS            VSPI_HOST
#define SPI_MOSI_PIN            GPIO_NUM_23    
#define SPI_MISO_PIN            GPIO_NUM_19   
#define SPI_SCLK_PIN            GPIO_NUM_18  
#define SPI_CLOCK_HZ            1000000         // 1MHz: well within MAX31865's 5MHz SPI ceiling, generous timing margin
#define RTD_CHANNEL_COUNT       16

/************************************************************************************************************************
RS-485 / MODBUS RTU - sensor node is the SLAVE, the S7-1200 PLC is the master
****************************************************************************/
#define MODBUS_UART_NUM         UART_NUM_1    
#define MODBUS_TX_PIN           GPIO_NUM_17   
#define MODBUS_RX_PIN           GPIO_NUM_16   
#define RS485_EN_PIN            GPIO_NUM_2      // automatic RS485 power-switching enable
#define MODBUS_BAUD_RATE        9600            // Revisit after latency budget
#define MODBUS_SLAVE_ID         1               // This sensor node's Modbus slave address

/***********************************************************************************************************************
CAN BUS - sensor node is a broadcast-only publisher, no ACK expected
*******************************************************************/
#define CAN_TX_PIN              GPIO_NUM_25    
#define CAN_RX_PIN              GPIO_NUM_26  
#define CAN_EN_PIN              GPIO_NUM_4      // CAN transceiver power-switching enable

/***********************************************************************************************************************
MISC I/O
*******/
#define BUTTON_PIN               GPIO_NUM_14     // service button 
#define STATUS_LED_PIN           GPIO_NUM_15     // Status_LED net
#define POWER_PIN                GPIO_NUM_27     // latching power-switch shutdown line

#endif /* MAIN_PINOUT_H_ */
