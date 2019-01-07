/*!
 *****************************************************************************
  @file adi_sensor_errors.h

  @brief Sensor error codes.

  @details
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

  THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-
  INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF
  CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/


#ifndef ADI_SENSOR_ERRORS_H
#define ADI_SENSOR_ERRORS_H


#include <stdint.h>
#include <assert.h>


namespace adi_sensor_swpack
{

/*!
    @addtogroup common_errors Error Handling
    @ingroup common
    @{

    @brief    Result codes and error macros.

    @details  Contains supported sensor errors and macros for packing and
              unpacking these error codes. Also contains macros for printing
              and asserting.
*/

/*! Given a #SENSOR_ERROR_TYPE type and a corresponding driver error code, format the error value. */
#define SET_SENSOR_ERROR(type, error) (uint32_t)((error) | (type << 24ul))

/*! Extract the #SENSOR_ERROR_TYPE from the #SENSOR_RESULT */
#define GET_SENSOR_ERROR_TYPE(result) ((SENSOR_ERROR_TYPE) ((result & 0xFF000000ul) >> 24ul))

/*! Extract the driver error code from the #SENSOR_RESULT - the type will be a ADI_XXX_RESULT enumeration */
#define GET_DRIVER_ERROR_CODE(result) (result & 0x00FFFFFFul)

/*! Sensor API result code */
typedef uint32_t SENSOR_RESULT;

/*! Sensor error types */
typedef enum
{
  SENSOR_ERROR_NONE   = 0u,

  /* Peripherals */
  SENSOR_ERROR_DMA    = 1u,        /*!< DMA driver reported an error           */
  SENSOR_ERROR_FLASH  = 2u,        /*!< Flash driver reported an error         */
  SENSOR_ERROR_GPIO   = 3u,        /*!< GPIO driver reported an error          */
  SENSOR_ERROR_I2C    = 4u,        /*!< I2C driver reported an error           */
  SENSOR_ERROR_PWR    = 5u,        /*!< Power driver reported an error         */
  SENSOR_ERROR_SPI    = 6u,        /*!< SPI driver reported an error           */
  SENSOR_ERROR_SPORT  = 7u,        /*!< SPORT driver reported an error         */
  SENSOR_ERROR_UART   = 8u,        /*!< UART driver reported an error          */

  /* Sensors */
  SENSOR_ERROR_ADC    = 9u,        /*!< ADC sensor reported an error           */
  SENSOR_ERROR_AXL    = 10,        /*!< Accelerometer sensor reported an error */
  SENSOR_ERROR_C02    = 11u,       /*!< C02 sensor reported an error           */
  SENSOR_ERROR_GAS    = 12u,       /*!< Gas sensor reported an error           */
  SENSOR_ERROR_LIGHT  = 13u,       /*!< Visual Light sensor reported an error  */
  SENSOR_ERROR_PH     = 14u,       /*!< PH sensor reported an error            */
  SENSOR_ERROR_TEMP   = 15u,       /*!< Temperature sensor reported an error   */

} SENSOR_ERROR_TYPE;


#ifdef ADI_DEBUG
/*! Assert macro which maps to standard C assert in debug, nothing in release */
#define ASSERT(x) assert(x)
#else
/*! Assert macro which maps to standard C assert in debug, nothing in release */
#define ASSERT(x)
#endif

/*! Print the sensor error code by breaking it into the two parts using the macros defined above */
#define PRINT_SENSOR_ERROR(func, result) func("Sensor Error = %X\tDriver Error = %X\r\n", (int) GET_SENSOR_ERROR_TYPE(result), (int) GET_DRIVER_ERROR_CODE(result))

}

#endif /* ADI_SENSOR_ERRORS_H */
