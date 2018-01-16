/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for communication part.
*   @version  V0.3
*   @author   ADI
*   @date     March 2016
*  @par Revision History:
*  - V0.1, September 2015: initial version.
*  - V0.2, October 2015: changed used SPI channel to SPI1 and added revision history.
*  - V0.3, March 2016: changed SPI pins initialization.
*
*******************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
**/


/***************************** Include Files **********************************/
#include <stdio.h>
#include <stdint.h>

#include "Communication.h"

/************************* Functions Definitions ******************************/

/**
   @brief Writes a data, a command or a register to the LCD or to ACC via SPI.

   @param ui8address - ACC register address
   @param ui8Data - value to be written
   @enMode ui8Data - write mode

   @return none

**/
void SPI_Write(uint8_t ui8address, uint8_t ui8Data, enWriteData enMode)
{
   if(enMode != SPI_WRITE_REG) {

      digitalWrite(CSLCD_PIN,LOW);  /* Select LCD */

      if(enMode == SPI_WRITE_DATA) {

          digitalWrite(A0LCD_PIN,HIGH);  /* Select to send data */

      } else if(enMode == SPI_WRITE_COMMAND) {

          digitalWrite(A0LCD_PIN,LOW);   /* Select to send command */
      }

      SPI.transfer(ui8Data);

      digitalWrite(CSLCD_PIN,HIGH);   /* Deselect LCD */

   } else {

      digitalWrite(CSACC_PIN,LOW); /* Select accelerometer */

      SPI.transfer(COMM_WRITE);       /* Send write command */

      SPI.transfer(ui8address);          /* Send register address */

      SPI.transfer(ui8Data);             /* Send value to be written */

      digitalWrite(CSACC_PIN,HIGH);         /* Deselect accelerometer */
   }

}

/**
   @brief Reads a specified register or two registers address in the accelerometer via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint16_t SPI_Read(uint8_t ui8address, enRegsNum enRegs)
{

   uint16_t  ui16Result = 0;

   uint8_t ui8value;
   uint16_t ui16valueL;
   uint16_t ui16valueH;

   digitalWrite(CSACC_PIN,LOW);      /* Select accelerometer */

   SPI.transfer(COMM_READ);       /* Send read command */

   SPI.transfer(ui8address);       /* Send register address */

   if (enRegs == SPI_READ_ONE_REG) {

      /* Read the register value */
      ui8value =  SPI.transfer(0xAA); 

      ui16Result = (uint16_t)ui8value;   /* Set read result*/

   } else {
    
      /* Read the two register values */
      ui16valueL =  SPI.transfer(0xAA); 
      ui16valueH =  SPI.transfer(0xAA); 

      ui16Result = (uint16_t)((ui16valueH << 8) | ui16valueL); /* Set read result*/
   }

   /* Deselect accelerometer */
   digitalWrite(CSACC_PIN,HIGH);

   return ui16Result;
}



