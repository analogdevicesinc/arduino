/*!
 *****************************************************************************
  @file adi_sensor_packet.h

  @brief Packet structure for sending data to the Android application.

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


/**
    @defgroup common Common
*/


#ifndef ADI_SENSOR_PACKET_H
#define ADI_SENSOR_PACKET_H


#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

namespace adi_sensor_swpack
{

#ifdef __cplusplus
}
#endif


/*!
    @addtogroup adi_packet Packet Definitions
    @ingroup common
    @{

    @brief Packet definitions.
    @details  Shared header file for common packet defintions
*/


/*! Takes in a #ADI_PACKET_TYPE and a user defined sensor id and formats it for a header */
#define ADI_SET_HEADER(packet_type, id) ((id & 0x7Fu) | (packet_type << 7))

/*! Maximum string size. This is: size of the data packet - header - 1 byte size = 20 - 6 - 1 = 13 */
#define ADI_MAX_STRING_SIZE (13u)

/*!
   @enum ADI_PACKET_TYPE

   @brief Packet type IDs

   @note These correlate with the Android application. So any
         changes here must be reflected in SharedDefines.java
*/
typedef enum
{
  /*!< Indicates that the current packet is a registration packet */
  ADI_REGISTRATION_PACKET_TYPE     = 0x00u,
  /*!< Indicates that the current packet is a data packet */
  ADI_DATA_PACKET_TYPE             = 0x01u,
} ADI_PACKET_TYPE;

/*!
   @enum ADI_SENSOR_TYPE

   @brief Senor type IDs for the various types of sensors.

   @note These correlate with the Android application. So any
         changes here must be reflected in SharedDefines.java
*/
typedef enum
{
  ADI_GENERIC_TYPE         	    = 0x00u, /*!< Generic sensor type */
  ADI_ACCELEROMETER_2G_TYPE     = 0x01u, /*!< Accelerometer (2G) type */
  ADI_CO_TYPE                   = 0x02u, /*!< Carbon monoxide sensor type */
  ADI_TEMPERATURE_TYPE          = 0x03u, /*!< Temperature sensor type */
  ADI_VISIBLELIGHT_TYPE         = 0x04u, /*!< Visible light sensor type */
  ADI_PRINTSTRING_TYPE          = 0x05u, /*!< Print string sensor type */
  ADI_ACCELEROMETER_4G_TYPE     = 0x0Bu, /*!< Accelerometer (4G) type */
  ADI_ACCELEROMETER_8G_TYPE     = 0x0Cu, /*!< Accelerometer (8G) type */
} ADI_SENSOR_TYPE;

/*!
   @enum ADI_DATA_TYPE_KEY

   @brief This is a key for the android app to determine how a data
          field should be interpreted.

   @note These correlate with the Android application. So any
         changes here must be reflected in SharedDefines.java
*/
typedef enum
{
  ADI_BYTE_TYPE                 = 0x00u, /*!< 1 byte integer data bytes */
  ADI_SHORT_TYPE                = 0x01u, /*!< 2 byte integer data bytes */
  ADI_INT_TYPE                  = 0x02u, /*!< 4 byte integer data bytes */
  ADI_LONG_TYPE                 = 0x03u, /*!< 8 byte integer data bytes */
  ADI_FLOAT_TYPE                = 0x04u, /*!< 4 byte float data byte */
  ADI_DOUBLE_TYPE               = 0x05u, /*!< 8 byte float data byte */
  ADI_CHAR_TYPE                 = 0x06u, /*!< 1 byte char data byte */
} ADI_DATA_TYPE_KEY;


/*!
    @struct ADI_STRING_DATA

    @brief  data structure for the #ADI_PRINTSTRING_TYPE

*/
#pragma pack(push)
#pragma pack(1)
typedef struct
{
  uint8_t           nStringSize;                       /*!< Size of the string to send. Must not be larger than 13 bytes. */
  uint8_t           aStringData[ADI_MAX_STRING_SIZE];  /*!< String to send.                                               */

} ADI_STRING_DATA;
#pragma pack(pop)

/*!
    @struct ADI_ACCELEROMETER_DATA

    @brief  Data structure for the #ADI_ACCELEROMETER_2G_TYPE, #ADI_ACCELEROMETER_4G_TYPE
            and #ADI_ACCELEROMETER_8G_TYPE.

*/
#pragma pack(push)
#pragma pack(1)
typedef struct
{
  uint8_t           aData_X[2];        /*!< Accelerometer X value */
  uint8_t           aData_Y[2];        /*!< Accelerometer Y value */
  uint8_t           aData_Z[2];        /*!< Accelerometer Z value */

} ADI_ACCELEROMETER_DATA;
#pragma pack(pop)

/*!
    @struct ADI_VISUAL_LIGHT_DATA

    @brief  Data structure for the #ADI_VISIBLELIGHT_TYPE data type.

*/
#pragma pack(push)
#pragma pack(1)
typedef struct
{
  float           fData_Red;        /*!<  Red Photodiode in lux   */
  float           fData_Green;      /*!<  Green Photodiode in lux */
  float           fData_Blue;       /*!<  Blue Photodiode in lux  */

} ADI_VISUAL_LIGHT_DATA;
#pragma pack(pop)

/*!
    @struct ADI_DATA_PACKET

    @brief  packet structure to send data to the Android application

*/
#pragma pack(push)
#pragma pack(1)
typedef struct
{
  uint8_t                     nPacketHeader;      /*!< Packet header has the first bit set to 0x1u to indicated this is a
                                                        data packet. The rest of the 7 bits are used for the sensor id.
                                                        This should be a unique identifier of the sensor instance.    */
  ADI_SENSOR_TYPE             eSensorType;        /*!< Sensor type                                                  */
  uint8_t                     aTimestamp[4];      /*!< Timestamp value                                              */
  uint8_t                     aPayload[14];       /*!< Data payload. This varies based on the sensor sending data   */
} ADI_DATA_PACKET;
#pragma pack(pop)

/*!
    @struct ADI_REGISTRATION_PACKET

    @brief  packet structure to send a registration packet to the Android application

*/
#pragma pack(push)
#pragma pack(1)
typedef struct
{
  uint8_t             nPacketHeader;          /*!< Packet header has the first bit set to 0x0u to indicated this is a
                                                     registration packet. The rest of the 7 bits are used for the sensor id.
                                                     This should be a unique identifier of the sensor instance.  */
  ADI_SENSOR_TYPE     eSensorType;            /*!< Sensor type */
  uint8_t             nNumDataTypes;          /*!< Number of data types  */
  ADI_DATA_TYPE_KEY   aDataTypeKey[17];       /*!< For each data type (amount specified by nNumDataTypes), indicate
                                                     how the Android application will interpret this data */
} ADI_REGISTRATION_PACKET;
#pragma pack(pop)

/*! @} */

#ifdef __cplusplus
extern "C" {
#endif

}

#ifdef __cplusplus
}
#endif

#endif /* ADI_SENSOR_PACKET_H */
