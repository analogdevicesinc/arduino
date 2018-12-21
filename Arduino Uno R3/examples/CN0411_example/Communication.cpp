/*!
 *******************************************************************************
 * @file:    Communication.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *------------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
NON-INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/***************************** Include Files **********************************/

#include <Arduino.h>
#include <SPI.h>
#include "Communication.h"
#include <stdio.h>
#include <stdint.h>

/********************************* Global data ********************************/


uint8_t           uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t           uart_tx_buffer[UART_TX_BUFFER_SIZE];

uint8_t         uart_rcnt, uart_tpos, uart_tcnt;
uint8_t         uart_cmd, uart_tbusy;

/**
   @brief SPI initialization

   @return none

**/
void SPI_Init()
{
	SPI.begin();

	pinMode(CS_AD7124_PIN, OUTPUT);
	digitalWrite(CS_AD7124_PIN, HIGH);
	pinMode(CS_AD5683_PIN, OUTPUT);
	digitalWrite(CS_AD5683_PIN, HIGH);
}

/**
   @brief Reads a specified register address in the converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - register address
   @param ui8_nr_bytes - register number of bytes

   @return reading result

**/
int32_t SPI_Read(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
		 uint8_t ui8_nr_bytes)
{

	int32_t ret = 0;
	/*Clear Slave based on ID */

	switch(ui8_slave_id) {
	case 0:
		digitalWrite(CS_AD7124_PIN, LOW);
		SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
		break;
	case 1:
		digitalWrite(CS_AD5683_PIN, LOW);
		SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE2));
		break;
	}

	SPI.transfer(ui8_buffer, ui8_nr_bytes);
	SPI.endTransaction();

	/*Set Slave based on ID */
	switch(ui8_slave_id) {
	case 0:
		digitalWrite(CS_AD7124_PIN, HIGH);
		break;
	case 1:
		digitalWrite(CS_AD5683_PIN, HIGH);
		break;
	}

	if(ui8_nr_bytes == 0)
		ret = -1;

	return ret;

}

/**
   @brief Writes a register to the Converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - ACC register address
   @param ui32_data - value to be written
   @param ui8_nr_bytes - nr of bytes to be written

   @return none

**/
int32_t SPI_Write(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
		  uint8_t ui8_nr_bytes)
{
	int32_t ret = 0;

	if (ui8_nr_bytes > 4) {
		ui8_nr_bytes = 4;
	}

	/*Clear Slave based on ID */
	switch(ui8_slave_id) {
	case 0:

		digitalWrite(CS_AD7124_PIN, LOW);
		SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
		break;
	case 1:

		digitalWrite(CS_AD5683_PIN, LOW);
		SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE2));
		break;
	}

	SPI.transfer(ui8_buffer, ui8_nr_bytes);
	SPI.endTransaction();

	/*Set Slave based on ID */
	switch(ui8_slave_id) {
	case 0:
		digitalWrite(CS_AD7124_PIN, HIGH);
		break;
	case 1:
		digitalWrite(CS_AD5683_PIN, HIGH);
		break;
	}

	if(ui8_nr_bytes == 0)
		ret = -1;

	return ret;

}

/**
  @brief UART initialization

  @param lBaudrate - Baud rate value (see UrtLib.c for values)
  @param iBits - Number of UART bits sent (see UrtLib.c for values)

  @return none

**/

void UART_Init(long lBaudrate, int iBits)
{
	Serial.begin(lBaudrate);
}

/**
  @brief Writes one character to UART.

  @param data - Character to write.
  @param mode - Write mode

  @return UART_SUCCESS or error code.

**/
void UART_WriteChar(char data, enWriteData mode)
{
	Serial.write(data);
}

/**
  @brief Writes string to UART.

  @param string - string to write.

  @return UART_SUCCESS or error code.

**/
void UART_WriteString(char *string)
{
	Serial.write(string);
}

/**
  @brief Read character from UART.

  @param data - data that is received.

  @return none

**/
void UART_ReadChar(char *data)
{
	*data = Serial.read();         /* Read character from UART */
}

