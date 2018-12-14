/***************************************************************************//**
     @file   adi_cn0410.cpp
     @brief  Implementation of CN0410 dac interface.
     @author Mircea Caprioru (mircea.caprioru@analog.com)
********************************************************************************
   Copyright 2018(c) Analog Devices, Inc.

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    - Neither the name of Analog Devices, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
    - The use of this software may or may not infringe the patent rights
      of one or more patent holders.  This license does not release you
      from the requirement that you obtain separate licenses from these
      patent holders to use this software.
    - Use of the software either in source or binary form, must be run
      on or directly connected to an Analog Devices Inc. component.

   THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
   IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "adi_cn0410.h"

CN0410::CN0410()
{
  /* Constructor for CN0410 */
}

int8_t CN0410::Init()
{

  /* Initialize SPI driver */
  SPI.begin();
  SPI.setDataMode(SPI_MODE2); //CPHA = 0, CPOL = 1    MODE = 2

  /* Setup LDAC & Reset pins */
  pinMode(LDAC_PIN, OUTPUT);
  digitalWrite(LDAC_PIN, LOW);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, HIGH);

  return 0;
}

void CN0410::Reset()
{
  /* Reset all led values */
  this->SendCommand(AD5686_WRITE_UPDATE, AD5686_DAC_A, 0x00);
  this->SendCommand(AD5686_WRITE_UPDATE, AD5686_DAC_B, 0x00);
  this->SendCommand(AD5686_WRITE_UPDATE, AD5686_DAC_C, 0x00);
  this->SendCommand(AD5686_WRITE_UPDATE, AD5686_DAC_D, 0x00);
}

void CN0410::UpdateDAC()
{
  /* Update the DAC register by pulsing the LDAC pin */
  digitalWrite(LDAC_PIN, HIGH);
  digitalWrite(LDAC_PIN, LOW);
  digitalWrite(LDAC_PIN, HIGH);
}

int8_t CN0410::SendCommand(uint8_t u8Command,
                           uint8_t u8Channel, uint16_t u16Value)
{
  uint8_t u8Buffer[3];

  u8Buffer[0] = ((u8Command & 0x0F) << 4) + (u8Channel & 0x0F);
  u8Buffer[1] = u16Value >> 8;
  u8Buffer[2] = u16Value & 0xFF;

  digitalWrite(SYNC_PIN, LOW);

  SPI.transfer(u8Buffer, 3);

  digitalWrite(SYNC_PIN, HIGH);

  return 0;
}

uint32_t CN0410::ReadBack(uint8_t u8DacChannelAddr)
{
  uint32_t u32ChannelValue;
  uint8_t rxBuffer[3] = {0, 0, 0};
  uint8_t txBuffer[3] = {0, 0, 0};

  this->SendCommand(AD5686_SET_READBACK, u8DacChannelAddr, 0x00);

  digitalWrite(SYNC_PIN, LOW);

  for (int i = 0; i < 3; i++)
    rxBuffer[i] = SPI.transfer(txBuffer[i]);

  digitalWrite(SYNC_PIN, HIGH);

  u32ChannelValue = (rxBuffer[0] << 16) + (rxBuffer[1] << 8) + rxBuffer[2];

  return u32ChannelValue;
}

CN0410::~CN0410()
{
  Serial.println("CN0410 object has been destroyed"); 
}
