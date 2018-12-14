/***************************************************************************//**
     @file   adi_cn0410.h
     @brief  Implementation of CN0410 header.
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
#include <Arduino.h>
#include <SPI.h>

#ifndef SENSORS_ADI_CN0410_H_
#define SENSORS_ADI_CN0410_H_

#define SYNC_PIN	10
#define LDAC_PIN	7
#define RESET_PIN	6

enum ad5686_commands {
  AD5686_NO_OPERATION = 0,
  AD5686_WRITE_LDAC,
  AD5686_UPDATE,
  AD5686_WRITE_UPDATE,
  AD5686_POWER,
  AD5686_LDAC_MASK,
  AD5686_RESET,
  AD5686_ITERNAL_REFERENCE,
  AD5686_SET_DCEN = 0x08,
  AD5686_SET_READBACK,
  AD5686_DAISY_CHAIN = 0x0F
};

enum ad5686_dac_channels {
  AD5686_DAC_NONE = 0x00,
  AD5686_DAC_A = 0x01,
  AD5686_DAC_B = 0x02,
  AD5686_DAC_C = 0x04,
  AD5686_DAC_D = 0x08
};

class CN0410
{
  public:
    CN0410();

    int8_t Init();
    void Reset();
    void UpdateDAC();
    int8_t SendCommand(uint8_t u8Command, uint8_t u8Channel,
                       uint16_t u16Value);
    uint32_t ReadBack(uint8_t u8DacChannelAddr);
    ~CN0410();

};

#endif /* SENSORS_ADI_CN0410_H_ */
