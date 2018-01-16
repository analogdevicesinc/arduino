/**
******************************************************************************
*   @file     Communication.c
*   @brief    Source file for communication part.
*   @version  V0.1
*   @author   ADI
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
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
#include <string.h>

#include "Communication.h"


/********************************* Global data ********************************/
unsigned char i = 0;

uint8_t convFlag, daisyCh;

/**
   @brief SPI initialization
   @return none
**/
void SPI_Init(void)
{
    pinMode(AD7798_CS_PIN, OUTPUT); // Set CS pin for AD7798 as output
    digitalWrite(AD7798_CS_PIN, HIGH);

    pinMode(ADT7310_CS_PIN, OUTPUT); // Set CS pin for ADT7310 as output
    digitalWrite(ADT7310_CS_PIN, HIGH);

    pinMode(AD5270_CS_PIN, OUTPUT); // Set CS pin for ADT7310 as output
    digitalWrite(AD5270_CS_PIN, HIGH);

    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
    delay(1000);
    
}

/***************************************************************************
 * @brief Writes data to SPI.
 *
 * @param data - Write data buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
void SPI_Write(unsigned char* data, unsigned char bytesNumber, enChannels ch)
{

    uint8_t count = 0;

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 digitalWrite(AD7798_CS_PIN, LOW);
                break;
             case ADT7310:
                 digitalWrite(ADT7310_CS_PIN, LOW);
                break;
             default:
               {
                  if(daisyCh == 0)
                     digitalWrite(AD5270_CS_PIN, LOW);
               }
                break;
          }
    }


    for(count = 0;count < bytesNumber;count++)
    {
          SPI.transfer(data[count]);  // write instruction
    }

   

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 digitalWrite(AD7798_CS_PIN, HIGH);
                break;
             case ADT7310:
                 digitalWrite(ADT7310_CS_PIN, HIGH);
                break;
             default:
               {
                  if(daisyCh == 0)
                     digitalWrite(AD5270_CS_PIN, HIGH);
               }
                break;
          }
    }

}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
void SPI_Read(unsigned char* data, unsigned char bytesNumber, enChannels ch)
{

   unsigned char writeData[4]  = {0, 0, 0, 0};
   unsigned char count          = 0;

    for(count = 0;count <= bytesNumber;count++)
    {
        if(count == 0)
           writeData[count] = data[count];
        else
           writeData[count] = 0xAA;    /* dummy value */
    }

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 digitalWrite(AD7798_CS_PIN, LOW);
                break;
             case ADT7310:
                 digitalWrite(ADT7310_CS_PIN, LOW);
                break;
             default:
                 digitalWrite(AD5270_CS_PIN, LOW);
                break;
          }
    }

    SPI.transfer(writeData[0]);

    for(count = 1; count < bytesNumber + 1;count++)
    {
        data[count - 1] = SPI.transfer(writeData[count]);
    }

    if(convFlag == 0){

          switch(ch){
             case AD7798:
                 digitalWrite(AD7798_CS_PIN, HIGH);
                break;
             case ADT7310:
                 digitalWrite(ADT7310_CS_PIN, HIGH);
                break;
             default:
                digitalWrite(AD5270_CS_PIN, HIGH);
                break;
          }
    }
}

