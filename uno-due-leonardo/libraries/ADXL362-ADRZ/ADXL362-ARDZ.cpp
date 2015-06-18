/*
  ADXL362.cpp - Library for ADXL362 - Micropower Accelerometer Sensor
  Created by Analog Devices Inc. - Circuits from the Lab, May 2015.
*/

#include <Arduino.h>
#include <SPI.h>
#include "ADXL362-ARDZ.h"

/******************************************************************************/
/********************************* Definitions ********************************/
/******************************************************************************/
// Accelerometer write command
#define COMM_WRITE         0x0A

// Accelerometer read command
#define COMM_READ          0x0B

// Accelerometer registers addresses
#define STATUS_REG         0x0B
#define XDATA_L_REG        0x0E
#define YDATA_L_REG        0x10
#define ZDATA_L_REG        0x12
#define TEMP_L_REG         0x14
#define SOFT_RESET_REG     0x1F
#define THRESH_ACT_L       0x20
#define THRESH_ACT_H       0x21
#define TIME_ACT           0x22
#define THRESH_INACT_L     0x23
#define THRESH_INACT_H     0x24
#define TIME_INACT_L       0x25
#define TIME_INACT_H       0x26
#define ACT_INACT_CTL      0x27
#define FIFO_CONTROL       0x28
#define INTMAP1            0x2A
#define INTMAP2            0x2B
#define FILTER_CTL     	   0x2C
#define POWER_CTL_REG      0x2D

// Accelerometer scan interval in ms
#define SCAN_SENSOR_TIME   100

// Activity threshold value
#define ACT_VALUE          50

// Inactivity threshold value
#define INACT_VALUE        50

// Activity timer value in ms
#define ACT_TIMER          100

// Inactivity timer value in seconds
#define INACT_TIMER        5

unsigned int	uireadValue = 0;
unsigned int	uihighReadValue = 0;
unsigned int	uilowReadValue = 0;


ADXL362class ADXL362;

void ADXL362class::ADXL362_SPI_Configuration(void)
{
	SPI.setBitOrder(MSBFIRST);          		//  MSB to be sent first
	SPI.setDataMode(SPI_MODE0);         		//  Set for clock rising edge, clock idles low
	SPI.setClockDivider(SPI_CLOCK_DIV128);		//  Set clock divider (optional)
	delay(100);
}

void ADXL362class::SensorINIT(void)
{	
	// do a software reset of the ADXL362
	ADXL362.SensorWriteOneReg(SOFT_RESET_REG, 0x52);
	delay(1000);
	
	// sets the activity threshold for the ADXL362
	ADXL362.SensorWriteOneReg(THRESH_ACT_L, (ACT_VALUE & 0xFF));			
	ADXL362.SensorWriteOneReg(THRESH_ACT_H, ACT_VALUE >> 8);
	
	// sets the activity time for the ADXL362
	ADXL362.SensorWriteOneReg(TIME_ACT, (ACT_TIMER / 10));
	
	// sets the inactivity threshold for the ADXL362
	ADXL362.SensorWriteOneReg(THRESH_INACT_L, (INACT_VALUE & 0xFF));		
	ADXL362.SensorWriteOneReg(THRESH_INACT_H, INACT_VALUE >> 8);
	
	// sets the inactivity time for the ADXL362
	ADXL362.SensorWriteOneReg(TIME_INACT_L, ((INACT_TIMER * 100) & 0xFF));
	ADXL362.SensorWriteOneReg(TIME_INACT_H, (INACT_TIMER * 100) >> 8);
	
	// sets the activity/inactivity control reg for the ADXL362
	ADXL362.SensorWriteOneReg(ACT_INACT_CTL, 0x3F);
	
	// sets the interrupt control sources for the ADXL362 (looking for awake)
	ADXL362.SensorWriteOneReg(INTMAP1, 0x40);
}

/***************************************************************************//**
 * @brief The function turns on accelerometer measurement mode.
 *
 * @return none.
*******************************************************************************/
void ADXL362class::StartSensor(void)
{
   uint8_t ui8temp;
   
   // Read POWER_CTL register, before modifying it
   ui8temp = ADXL362.SensorReadOneReg(POWER_CTL_REG);

   // Set measurement bit in POWER_CTL register
   ui8temp = ui8temp | 0x02;

   // Write the new value to POWER_CTL register
   ADXL362.SensorWriteOneReg(POWER_CTL_REG, ui8temp);
}
/***************************************************************************//**
 * @brief The function puts the accelerometer into standby mode.
 *
 * @return none.
*******************************************************************************/
void ADXL362class::StopSensor(void)
{
   uint8_t ui8temp;

   // Read POWER_CTL register, before modifying it
   ui8temp = ADXL362.SensorReadOneReg(POWER_CTL_REG);

   // Clear measurement bit in POWER_CTL register
   ui8temp = ui8temp & 0xFC;

   // Write the new value to POWER_CTL register
   ADXL362.SensorWriteOneReg(POWER_CTL_REG, ui8temp);
}
/***************************************************************************//**
 * @brief The function reads the accelerometer data.
 *
 * @return none.
*******************************************************************************/
void ADXL362class::ScanSensor(void)
{
	delay(SCAN_SENSOR_TIME);

    i16SensorX = ADXL362.SensorReadTwoReg(XDATA_L_REG);

    i16SensorY = ADXL362.SensorReadTwoReg(YDATA_L_REG);

    i16SensorZ = ADXL362.SensorReadTwoReg(ZDATA_L_REG);

    i16SensorT = ADXL362.SensorReadTwoReg(TEMP_L_REG);

}
/******************************************************************************/
/*************************** Static Functions *********************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The function writes a value to a register in the accelerometer.
 *
 * @param address - register address;
 * @param value - value to be written.
 *
 * @return The value read.
*******************************************************************************/
void ADXL362class::SensorWriteOneReg(byte address, byte value)
{
	digitalWrite(ADXL362_SS,LOW);
	SPI.transfer(COMM_WRITE);
	SPI.transfer(address);
	SPI.transfer(value);
	digitalWrite(ADXL362_SS,HIGH);
	delay(100);
}
/***************************************************************************//**
 * @brief The function reads a specified register address in the accelerometer.
 *
 * @param address - register address.
 *
 * @return The value read.
*******************************************************************************/
unsigned int ADXL362class::SensorReadOneReg(byte address)
{
	uireadValue = 0;
	
	digitalWrite(ADXL362_SS,LOW);
	SPI.transfer(COMM_READ);
	SPI.transfer(address);
	uireadValue = SPI.transfer(0x00);
	digitalWrite(ADXL362_SS,HIGH);
	delay(100);
	return uireadValue;
}
/***************************************************************************//**
 * @brief Reads a 16-bit values from two registers of the accelerometer.
 *
 * @param address - the address of the first register.
 *
 * @return The values read as a 16-bit unsigned integer.
*******************************************************************************/
unsigned int ADXL362class::SensorReadTwoReg(byte address)
{
	uireadValue = 0;
	uihighReadValue = 0;
	uilowReadValue = 0;
	
	digitalWrite(ADXL362_SS,LOW);
	SPI.transfer(COMM_READ);
	SPI.transfer(address);
	uilowReadValue = SPI.transfer(0x00);
	uihighReadValue = SPI.transfer(0x00);
	digitalWrite(ADXL362_SS,HIGH);
	delay(100);
	uireadValue = ((uihighReadValue<<8) | uilowReadValue);
	return uireadValue;
}