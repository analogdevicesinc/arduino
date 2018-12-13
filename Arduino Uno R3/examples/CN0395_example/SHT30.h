/*!
 *****************************************************************************
   @file:    SHT30.h
   @brief:   SHT30 sensor handling
   @version: $Revision$
   @date:    $Date$
  -----------------------------------------------------------------------------

  Copyright (c) 2017 Analog Devices, Inc.

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

#ifndef SHT30_H_
#define SHT30_H_

#include <Arduino.h>

#define SHT30_ADDR                     (uint8_t)0x45 //>> 1 ADDR pin connected to VDD

#define SHT30_MEAS_HIGHREP_STRETCH     0x2C06
#define SHT30_MEAS_MEDREP_STRETCH      0x2C0D
#define SHT30_MEAS_LOWREP_STRETCH      0x2C10
#define SHT30_MEAS_HIGHREP             0x2400
#define SHT30_MEAS_MEDREP              0x240B
#define SHT30_MEAS_LOWREP              0x2416

#define SHT30_MEAS_HIGHREP_PERIODIC    0x2737
#define SHT30_FETCH_DATA               0xE000

#define SHT30_SOFTRESET                0x30A2

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
void SHT30_Update(float *fTemp,  float *fHum);

#endif // SHT30_H_
