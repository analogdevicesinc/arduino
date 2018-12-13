/*!
 *****************************************************************************
   @file:    ADN8810.c
   @brief:   ADN8810 handling
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

#include <Arduino.h>
#include "ADN8810.h"
#include "AD7988.h"
#include "Communication.h"

/************************* Functions Definitions ******************************/

/**
   @brief Initializes the ADN8810

   @param None.

   @return None
**/
void ADN8810_Init(void)
{
  pinMode(IO5ADN8810_PIN, OUTPUT); /* Set IO5ADN8810 pin as output */
  pinMode(IO7ADN8810_PIN, OUTPUT); /* Set IO7ADN8810 pin as output */

  ADN8810_MasterPower(ADN8810_ENABLE); //  enable master power on IO5

  ADN8810_Reset(); // Reset DAC
}

/**
   @brief ADN8810 IOUT Factory Calibration for 50mA FS

   @param None.

   @return fGainCorrectionFactor - the calculated gain correction factor
**/
uint32_t ADN8810_FactoryCalibration(void)
{
  uint16_t ui16AdcData = 0;
  float fGainCorrectionFactor;
  uint8_t ui8UpperByte = 0;       // Data byte MSB
  uint8_t ui8LowerByte = 0;       // Data byte LSB
  uint16_t ui16DacInputCode = ADN8810_FULL_SCALE_OUT; // Full Scale for 9.9mA

  // build 12 bits of data + 4 bits of device address
  ui8UpperByte = (uint8_t)(((ui16DacInputCode & 0xF00) >> 8) | (ADN8810_ADR << 4)); // 4 address and 4 data bits
  ui8LowerByte = (uint8_t)(ui16DacInputCode & 0xFF); // 8 bits of data

  ADN8810_MasterPower(ADN8810_ENABLE);                        // enable master power on IO5
  digitalWrite(IO6ADG758_PIN, HIGH);                          // enable master power on IO6
  digitalWrite(CSADN8810_PIN, LOW);                           // Set CS_N_ARD to logic 0
  SPI_Write(ui8UpperByte, ui8LowerByte, SPI_WRITE_DAC_REG);   // Load FS code into ADN8810 DAC
  digitalWrite(CSADN8810_PIN, HIGH);                          // Set CS_N_ARD to logic 1
  digitalWrite(IN1ADG884_PIN, HIGH);                          // set IN1 to logic 1

  delay(50);                                                  // delay 50ms
  AD7988_ReadData(&ui16AdcData);                              // Read heater voltage ADC data

  fGainCorrectionFactor = (ADC_CODE_CALIBRATION / ui16AdcData) * 10000;   // Calculate gain correction factor

  return (uint32_t)fGainCorrectionFactor;
}

/**
   @brief Set ADN8810 current output

   @param fDesiredOutputCurrent - the desired IOUT current for ADN8810.

 *        *sMeasVar - pointer to the struct that contains all the relevant measurement variables

   @return -1 in case of error and 1 in case of success
**/
int ADN8810_SetOutput(float fDesiredOutputCurrent, sMeasurementVariables *sMeasVar)
{
  uint16_t ui16DacInputCode;
  // Adjust the current with the gain correction factor
  fDesiredOutputCurrent *= sMeasVar->K1;

  if (fDesiredOutputCurrent > ADN8810_IFS) {
    return -1;
  }
  else {
    ui16DacInputCode = ADN8810_CurrentToDataInputCalc(fDesiredOutputCurrent); // Convert input current to a DAC value

    uint8_t ui8UpperByte = 0;       /* Data byte MSB */
    uint8_t ui8LowerByte = 0;       /* Data byte LSB */

    /* build 12 bits of data + 4 bits of device address */
    ui8UpperByte = (uint8_t)(((ui16DacInputCode & 0xF00) >> 8) | (ADN8810_ADR << 4)); // 4 address and 4 data bits
    ui8LowerByte = (uint8_t)(ui16DacInputCode & 0xFF); // 8 bits of data

    SPI_Write(ui8UpperByte, ui8LowerByte, SPI_WRITE_DAC_REG); // Load code into ADN8810 DAC

    return 1;
  }
}


/**
  @brief Calculates the input code for ADN8810 IDAC, given a desired output current

  @param fCurrent - desired output current between 2.4uA and 9.9mA

  @return fCode - the value that will be received as input by the IDAC.
**/
uint16_t ADN8810_CurrentToDataInputCalc(float fCurrent)
{
  float fCode = 0;

  fCode = fCurrent / ADN8810_CURRENT_1LSB;

  return (uint16_t)fCode;
}

/**
  @brief Enable/Disable power for ADN8810 by setting/resetting IO5

  @param status - Enable or Disable

  @return None.
**/
void ADN8810_MasterPower(enADN8810Status status)
{
  if (status == ADN8810_ENABLE)
    digitalWrite(IO5ADN8810_PIN, HIGH); // enable master power on IO5
  else if (status == ADN8810_DISABLE)
    digitalWrite(IO5ADN8810_PIN, LOW);
}

/**
  @brief Reset ADN8810

  @param None

  @return None.
**/
void ADN8810_Reset(void)
{
  /* Reset DAC: Make RESET IO7 logic 0 */
  digitalWrite(IO7ADN8810_PIN, LOW);
  delay(1); // delay 1ms
  digitalWrite(IO7ADN8810_PIN, HIGH);
}
