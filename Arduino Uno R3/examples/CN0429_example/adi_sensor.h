/*!
 *****************************************************************************
  @file adi_sensor.h

  @brief Sensor class definition.

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

#ifndef ADI_BASE_SENSOR_H
#define ADI_BASE_SENSOR_H

#include "adi_sensor_packet.h"
#include "adi_sensor_errors.h"

namespace adi_sensor_swpack
{

/*!
   @class Sensor

   @brief Generic interface for all sensors.

 **/
class Sensor
{
  public:

    /**
       @brief    Initalizes the sensor and configures it.

       @return   SENSOR_RESULT

       @details  Initializes the sensor, opens and configures the underlying peripheral.
                 See sensor specific open member function for more details.
    */
    virtual SENSOR_RESULT   open() = 0;

    /**
       @brief    Starts sensor and enables the underlying device

       @return   SENSOR_RESULT

       @details  Start function's functionality varies from sensor to sensor. It enables
                 the device so that data acquisition could start. See sensor specific
                 start member function for more details.
    */
    virtual SENSOR_RESULT   start() = 0;

    /**
       @brief    Stops the sensor and disables the underlying device

       @return   SENSOR_RESULT

       @details  Stop function's functionality varies from sensor to sensor. It disables
                 the device and stops any pending data transfers. See sensor specific
                 stop member function for more details.
    */
    virtual SENSOR_RESULT   stop() = 0;

    /**
       @brief    Disables the sensor and closes underlying peripheral

       @return   SENSOR_RESULT

       @details  Close function disables the sensor and closes underlying peripheral.
                 See sensor specific close member function for more details.
    */
    virtual SENSOR_RESULT   close() = 0;

    /**
       @brief    Function returns the sensor identifier.

       @return   Returns the sensor ID.

       @details  Sensor ID is used to uniquiely idenfify a sensor in system. This is
                 used to pass information to the host application to identify the
                 source of the sensor data. Sensor ID could be same across different
                 types of the sensors but it should be unique with in a type.
    */
    uint32_t getID()
    {
      return m_sensor_id;
    }

    /**
       @brief    Function returns the sensor type.

       @return   Sensor type

       @details  Returns the sensor type. See ADI_SENSOR_TYPE enumeration for supported
                 sensor types.
    */
    ADI_SENSOR_TYPE getType()
    {
      return m_ADI_SENSOR_TYPE;
    }

    /**
       @brief    Function returns the sensor version.

       @return   Sensor version. A value of 0 is returned if sensor has no version
                 information.

       @details  Returns the sensor version. Sensor version contains the hardware
                 version information. A sensor may or may not have this information.
    */
    uint32_t getVersion(void)
    {
      return 0;
    }


    /**
       @brief    Function used to set the ID for the sensor.

       @param    sensorID : Sensor ID to be set.

       @return   none

       @details  Sensor ID is used to identify a sensor in the system. Host applications
                 uniquely identify a sensor based on its type and sensor id. Different
                 sensor types can have the same id.
    */
    void setID(const uint32_t sensorID)
    {
      m_sensor_id = sensorID;
    }

    /**
       @brief    Function used to set the type of the sensor

       @param    sensorType : Sensor type to be used.

       @return   none

       @details  Sensor type is used to identify the type of the sensor.ADI_SENSOR_TYPE
                 enumeration lists various supported sensor types. Host applications
                 uniquely identify a sensor based on its type and sensor id. This
                 function is typically implemented in the derived sensor classes.
    */
    void setType(const ADI_SENSOR_TYPE sensorType)
    {
      m_ADI_SENSOR_TYPE = sensorType;
    }


    /**
       @brief    Function used to set the version of the sensor

       @param    version : Sensor version to be used.

       @return   none

       @details  Sensor version holds the hardware version information of a sensor. This
                 function is typically implemented in the derived sensor classes.
    */
    void setVersion(const uint32_t version)
    {
    }

    /**
       @brief    Returns the last hardware error

       @return   Returns the hardware error.

       @details  If any of the object member functions fails because of an hardware
                 error this function has to be used to get the specific error.
    */
    uint32_t getLastHwError()
    {
      return 0;
    }

  protected:

    /**
       @brief    Sets the last hardware error

       @param    lastError : Last hardware error

       @return   none

       @details  Sensor drivers use this function to store the hardware error.
    */
    void setLastHwError(const uint32_t lastError)
    {
      
    }

  private:
    uint32_t    m_sensor_id;                     /*!< Sensor ID       */
    ADI_SENSOR_TYPE m_ADI_SENSOR_TYPE;           /*!< Sensor Type     */
};
}

#endif /* ADI_BASE_SENSOR_H */
