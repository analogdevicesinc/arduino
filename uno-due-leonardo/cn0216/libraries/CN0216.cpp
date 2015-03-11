/*
  CN0216.cpp - Library for CN0216 Embedded World Demo- Weighing Scale Module
  Created by Analog Devices Inc. - Circuits from the Lab, January 2015.
  
  Always need to start with a write to the communications register of AD7791:
  Here is the normal operation of the part without using the data ready pin 
  Write: 
		Mode Register = 0x10
		Filter Register = 0x20
  Read:
		Status Register = 0x08
		Data Register = 0x38  
*/

#include "Arduino.h"
#include "CN0216.h"
#include "SPI.h"

CN0216class CN0216;

void CN0216class::Configure(byte ADCmode,byte ADCrate,byte CN0216_ss_pin)
{
  CN0216_ss_pin = ss_pin;
  
  pinMode(CN0216_ss_pin,OUTPUT);
  digitalWrite(CN0216_ss_pin,HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);          //  MSB to be sent first
  SPI.setDataMode(SPI_MODE3);         //  Set for clock rising edge
  SPI.setClockDivider(SPI_CLOCK_DIV4);    //  Set clock divider (optional)
  
  //resets the registers
  digitalWrite(CN0216_ss_pin,LOW);
  SPI.transfer(byte_full); //0xFF
  SPI.transfer(byte_full); //0xFF
  SPI.transfer(byte_full); //0xFF
  SPI.transfer(byte_full); //0xFF
  digitalWrite(CN0216_ss_pin,HIGH);
  delay(100);
  //configure the ADC filter register
  digitalWrite(CN0216_ss_pin,LOW);
  SPI.transfer(0x20);
  SPI.transfer(byte_zero | ADCrate);
  digitalWrite(CN0216_ss_pin,HIGH);
  delay(100);
  //configure the ADC mode register
  digitalWrite(CN0216_ss_pin,LOW);
  SPI.transfer(0x10);
  SPI.transfer(ADCmode);
  digitalWrite(CN0216_ss_pin,HIGH);
  }

long CN0216class::Readdata()
{

	//checks the status register if device is ready
	do
	{
		digitalWrite(ss_pin,LOW);
		SPI.transfer(0x08);
		cRDY = SPI.transfer(byte_zero); //0x00
		digitalWrite(ss_pin,HIGH);
		//delay(200);
	}while(cRDY != 0x0C); // meaning that the ADC has a new conversion
  
    // read 24 bits of data
	digitalWrite(ss_pin,LOW);
	SPI.transfer(0x38);
	u8_ADCDATA2 = SPI.transfer(byte_zero); //0x00
	u8_ADCDATA1 = SPI.transfer(byte_zero); //0x00
	u8_ADCDATA0 = SPI.transfer(byte_zero); //0x00
	digitalWrite(ss_pin,HIGH);
  
	lADCDATA = ((long)u8_ADCDATA2<<16) | ((long)u8_ADCDATA1<<8) | u8_ADCDATA0;
	
	return lADCDATA;
}

void CN0216class::Calibrate_zero()
{
  int x = 0;
  ulADCZEROSCALE = 0;

  for(x=0;x<100;x++)
  {
  ulADCZEROSCALE += CN0216.Readdata();
  delay(200);
  }
  fADCZEROSCALE = ulADCZEROSCALE/100.0;
}

void CN0216class::Calibrate_fullscale()
{
  int x=0;
  ulADCFULLSCALE = 0;

  for(x=0;x<100;x++)
  {
  ulADCFULLSCALE += CN0216.Readdata();
  delay(200);
  }
  fADCFULLSCALE = ulADCFULLSCALE/100.0;
}

float CN0216class::Readweight(int fcal_wt_input)
{
	float fweight = 0;
	float fcodetoweight = fcal_wt_input / (fADCFULLSCALE-fADCZEROSCALE);
	
	fweight = (CN0216.Readdata()-fADCZEROSCALE) * fcodetoweight; 
	
	return fweight;
}