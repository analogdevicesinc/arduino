/*
  ADXL362.h - Library for ADXL362 - Accelerometer
  Created by Analog Devices Inc. - Circuits from the Lab, May 2015.
*/
#ifndef ADXL362-ARDZ_h
#define ADXL362-ARDZ_h

#include "Arduino.h"

#define SCAN_SENSOR_TIME    100 //used as a scan delay  
#define XDATA_L_REG        0x0E
#define YDATA_L_REG        0x10
#define ZDATA_L_REG        0x12
#define TEMP_L_REG         0x14
#define ADXL362_SS            7		// value of the CS pin assignment

extern int16_t i16SensorX;
extern int16_t i16SensorY;
extern int16_t i16SensorZ;
extern int16_t i16SensorT;

class ADXL362class
{
	public:
		void ADXL362_SPI_Configuration(void);
		void SensorINIT(void);
		void SensorWriteOneReg(byte address, byte value);
		unsigned int SensorReadOneReg(byte address);
		unsigned int SensorReadTwoReg(byte address);
		void ScanSensor(void);
		void StopSensor(void);
		void StartSensor(void);
		
	private:

};

extern ADXL362class ADXL362;

#endif
