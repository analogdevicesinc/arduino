/*!
 *****************************************************************************
   @file:    AD7988.c
   @brief:
   @version: $Revision$
   @date:    $Date$
  -----------------------------------------------------------------------------

  Copyright (c) 2018 Analog Devices, Inc.

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

  THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
  TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
  NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
  PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *****************************************************************************/

/***************************** Include Files **********************************/
#include <stdio.h>
#include <math.h>

#include <Arduino.h>
#include "AD7988.h"
#include "Communication.h"

/************************* Functions Definitions ******************************/

/**
   @brief Initializes the AD7988

   @param None.

   @return None
**/
void AD7988_Init(void)
{
  pinMode(IN1ADG884_PIN, OUTPUT);
  pinMode(A0ADG758_PIN, OUTPUT);
  pinMode(A1ADG758_PIN, OUTPUT);
  pinMode(A2ADG758_PIN, OUTPUT);
  pinMode(IO6ADG758_PIN, OUTPUT);
}

/**
   @brief Selects the desired gain resistor R1 to R5

   @param mode - desired mode of operation

   @return None
**/
void AD7988_SensorRangeSelect(enAD7988OpMode mode)
{
  uint8_t ui8A0Pin, ui8A1Pin, ui8A2Pin;

  ui8A0Pin = (mode & 0x1) >> 0;
  ui8A1Pin = (mode & 0x2) >> 1;
  ui8A2Pin = (mode & 0x4) >> 2;

  if (ui8A0Pin)
    digitalWrite(A0ADG758_PIN, HIGH);
  else
    digitalWrite(A0ADG758_PIN, LOW);

  if (ui8A1Pin)
    digitalWrite(A1ADG758_PIN, HIGH);
  else
    digitalWrite(A1ADG758_PIN, LOW);

  if (ui8A2Pin)
    digitalWrite(A2ADG758_PIN, HIGH);
  else
    digitalWrite(A2ADG758_PIN, LOW);
}

/**
   @brief Selects the desired mode of operation by enabling or disabling the power on the correct pins

   @param mode - desired mode of operation

   @return None
**/
void AD7988_SetOperationMode(enAD7988OpMode mode)
{
  if (mode == AD7988_RH_MODE) {
    digitalWrite(IN1ADG884_PIN, HIGH); // set IN1 to logic 1
    digitalWrite(IO6ADG758_PIN, LOW); // disable power on IO6
  }
  else {
    digitalWrite(IN1ADG884_PIN, LOW); // reset IN1
    digitalWrite(IO6ADG758_PIN, HIGH); // enable master power on IO6
    AD7988_SensorRangeSelect(mode);
  }
  delay(1);
}

/**
   @brief Reads AD7988 data from the SPI

   @param data - read data.

   @return None
**/
void AD7988_ReadData(uint16_t *data)
{
  *data =  SPI_Read();
}

/**
   @brief Converts the read ADC data to the corresponding voltage

   @param ui16Adcdata - ADC data read

   @return fAdcVoltage -  voltage according to the input data
**/
float AD7988_DataToVoltage(uint16_t ui16Adcdata)
{
  float fAdcVoltage = 0;

  fAdcVoltage = ((float)ui16Adcdata / (pow(2, 16) - 1) ) * ADC_VREF;

  return fAdcVoltage;
}
