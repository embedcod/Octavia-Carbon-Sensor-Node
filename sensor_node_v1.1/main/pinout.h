/*
 * pinout.h
 *
 *  Created on: 03/08/2026
 *		Author: Eric Mulwa
 *
 *  On-site POC build -- Octavia Carbon's hardware, pinout confirmed by Eric.
 */

#ifndef MAIN_PINOUT_H_
#define MAIN_PINOUT_H_

#include "driver/gpio.h"
#include "driver/uart.h"

/***********************************************************************************************************************
I2C BUS: shared bus to the single MCP23017 (16-bit GPIO expander), CS1-CS16
*************************************************************************/
#define I2C_SDA_PIN             GPIO_NUM_21
#define I2C_SCL_PIN             GPIO_NUM_22
#define I2C_PORT                I2C_NUM_0
#define I2C_CLK_HZ              400000          // 400kHz: fast enough for CS toggling between SPI words

// MCP23017 7-bit address. Default (A2/A1/A0 all tied to GND) is 0x20 -- CONFIRM against the
// board's address-pin strapping before relying on this.
//#define MCP23017_ADDR           0x20
#define MCP23017_ADDR           0x27

/************************************************************************************************************************
SPI BUS: shared bus to all 16 MAX31865ATP+T RTD amplifiers, CS1->CH1 ... CS16->CH16
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
#define MODBUS_TX_PIN           17
#define MODBUS_RX_PIN           16
#define MODBUS_RTS_PIN   		14
#define MODBUS_BAUD_RATE        19200
#define MODBUS_SLAVE_ID         1               // This sensor node's Modbus slave address

/***********************************************************************************************************************
CAN BUS - sensor node is a broadcast-only publisher, no ACK expected
*******************************************************************/
#define CAN_TX_PIN              GPIO_NUM_25
#define CAN_RX_PIN              GPIO_NUM_26

#endif /* MAIN_PINOUT_H_ */
