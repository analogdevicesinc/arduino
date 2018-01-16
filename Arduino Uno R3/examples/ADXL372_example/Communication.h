/**
******************************************************************************
*   @file     Communication.h
*   @brief    Header file for communication part
*   @version  V0.3
*   @author   ADI
*   @date     March 2016
*  @par Revision History:
*  - V0.1, September 2015: initial version.
*  - V0.2, October 2015: added missing comments and revision history.
*  - V0.3, March 2016: added pin configuration based on pin selection.
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
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "Arduino.h"
#include "SPI.h"
#include "adxl372.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
**************************** Internal types ************************************
********************************************************************************/

/* Write data mode */
typedef enum {
   SPI_WRITE_DATA = 1,            /* Write data to LCD */
   SPI_WRITE_COMMAND,               /* Write command to LCD */
   SPI_WRITE_REG                 /* Write ACC register */

} enWriteData;

typedef enum {
   SPI_READ_ONE_REG = 1,            /* Read one ACC register */
   SPI_READ_TWO_REG               /* Read two ACC registers */

} enRegsNum;


/*******************************************************************************
**************************** Internal definitions ******************************
********************************************************************************/

/* Accelerometer write command */
#define COMM_WRITE         0x0A

/* Accelerometer read command */
#define COMM_READ          0x0B

/* Unused address */
#define ADDR_NOT_USE       0x00

/* ADXL_CS_SEL pins */
#define ACC_PIN_CS1           1   /* Select pin 10 */
#define ACC_PIN_CS2           2   /* Select pin 8 */

/* ADXL_INT1_SEL pins */
#define INT1_ACC_PIN_A             3   /* Select INT1 */
#define INT1_ACC_PIN_B             4   /* Select INT1 */

/* ADXL_INT2_SEL pins */
#define INT2_ACC_PIN_A             5   /* Select INT1 */
#define INT2_ACC_PIN_B             6   /* Select INT1 */



/*******************************************************************************
**************************** Functions declarations *****************************
********************************************************************************/
int spi_write_then_read(adxl_spi_handle *spiTransfer,
                               unsigned char *txbuf,
                               unsigned n_tx,
                               unsigned char *rxbuf,
                               unsigned n_rx);


/*******************************************************************************
******************Configuration parameters(set by the user)*********************
********************************************************************************/
/* Select ADXL372 CS pin based on jumper P10 setting.
 * Available values:
 *    ACC_PIN_CS1
 *    ACC_PIN_CS2 */
#define ADXL_CS_SEL         ACC_PIN_CS1

/* Select ADXL372 INT1 pin based on jumper P11 setting.
 * Available values:
 *    INT1_ACC_PIN_A
 *    INT1_ACC_PIN_B */
#define ADXL_INT1_SEL        INT1_ACC_PIN_A

/* Select ADXL372 INT2 pin based on jumper P12 setting.
 * Available values:
 *    INT2_ACC_PIN_A
 *    INT2_ACC_PIN_B */
#define ADXL_INT2_SEL        INT2_ACC_PIN_A

/*********************Pins configuration (not set by the user)*******************/
/*** ACC CS pin configuration ***/
#if(ADXL_CS_SEL == ACC_PIN_CS1)
#define CS_PIN          10
#elif(ADXL_CS_SEL == ACC_PIN_CS2)
#define CS_PIN          8
#endif

/*** ACC INT pin configuration */
#if(ADXL_INT1_SEL == INT1_ACC_PIN_A)
#define INT1_ACC_PIN         7
#elif(ADXL_INT1_SEL == INT1_ACC_PIN_B)
#define INT1_ACC_PIN         6
#endif

#if(ADXL_INT2_SEL == INT2_ACC_PIN_A)
#define INT2_ACC_PIN         5
#elif(ADXL_INT2_SEL == INT2_ACC_PIN_B)
#define INT2_ACC_PIN         4
#endif


#ifdef __cplusplus
} // extern "C"
#endif

#endif /* _COMMUNICATION_H_ */
