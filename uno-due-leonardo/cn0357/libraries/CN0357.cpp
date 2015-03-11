/*
  CN0357.cpp - Library for CN0357 - Universal Toxic Gas Sensor
  Created by Analog Devices Inc. - Circuits from the Lab, December 2014.
  
  Always need to start with a write to the communications register of AD7790:
  Here is the normal operation of the part without using the data ready pin 
  Write: 
		Mode Register = 0x10
		Filter Register = 0x20
  Read:
		Status Register = 0x08
		Data Register = 0x38  
		
  Digital Potentiometer:
  
*/

#include "Arduino.h"
#include "CN0357.h"
#include "SPI.h"

CN0357class CN0357;

void CN0357class::Configure(byte ADCmode,byte ADCrate,byte ADC_ss, byte Rheostat_ss) //Configures the AD7790 ADC, set ODR and enable/disable buffers. Initializes the Rheostat
{
  pinMode(ADC_ss,OUTPUT);	//set CS pin for ADC
  pinMode(Rheostat_ss,OUTPUT);	//set CS pin for rheostat
  digitalWrite(ADC_ss,HIGH);
  digitalWrite(Rheostat_ss,HIGH);
  
  SPI.setBitOrder(MSBFIRST);          //  MSB to be sent first
  SPI.setDataMode(SPI_MODE3);         //  Set for clock rising edge
  SPI.setClockDivider(SPI_CLOCK_DIV128);    //  Set clock divider (optional)
  
  SPI.begin();
  delay(1000);

  //resets the registers
  digitalWrite(ADC_ss,LOW);
  SPI.transfer(byte_full); //0xFF
  SPI.transfer(byte_full); //0xFF
  SPI.transfer(byte_full); //0xFF
  SPI.transfer(byte_full); //0xFF
  digitalWrite(ADC_ss,HIGH);
  delay(1000);
  //configure the ADC
  digitalWrite(ADC_ss,LOW);	//mode register          
  SPI.transfer(0x10);
  SPI.transfer(ADCmode);
  digitalWrite(ADC_ss,HIGH);
  delay(100);
  digitalWrite(ADC_ss,LOW);	//filter register
  SPI.transfer(0x20);
  SPI.transfer(byte_zero | ADCrate);	
  digitalWrite(ADC_ss,HIGH);
  delay(100);
  digitalWrite(Rheostat_ss,LOW);	//write to rheostat control register
  SPI.transfer(0x1C);	
  SPI.transfer(0x02);	
  digitalWrite(Rheostat_ss,HIGH);
  Rheostat_SDO_HiZ(Rheostat_ss);
}

void CN0357class::Rheostat_SDO_HiZ(byte Rheostat_ss) //puts the rheostat SDO line in tristate/High-Z mode to minimize power and keep SDO line free
{
  digitalWrite(Rheostat_ss,LOW);	//prepare rheostat SDO pin to Hi-Z mode
  SPI.transfer(0x80);	
  SPI.transfer(0x01);	
  digitalWrite(Rheostat_ss,HIGH);
  delay(100);
  digitalWrite(Rheostat_ss,LOW);	//NOP command to place rheostat in Hi-Z
  SPI.transfer(byte_zero);	
  SPI.transfer(byte_zero);	
  digitalWrite(Rheostat_ss,HIGH);
}

unsigned int CN0357class::Readdata(byte ADC_ss) // read data from the ADC
{

	//	checks the status register if conversion is available
	do
	{
//		Serial.println("Polling Status Reg"); //debug
		digitalWrite(ADC_ss,LOW);
		SPI.transfer(0x08);
		cRDY = SPI.transfer(byte_zero); //0x00
		digitalWrite(ADC_ss,HIGH);
		delay(100);
	}while(cRDY != 0x08);
  
    // read 16 bits of data from data register
	digitalWrite(ADC_ss,LOW);
	SPI.transfer(0x38);
	u8_ADCDATA1 = SPI.transfer(byte_zero); //0x00
	u8_ADCDATA0 = SPI.transfer(byte_zero); //0x00
	digitalWrite(ADC_ss,HIGH);
  
	uiADCDATA = ((long)u8_ADCDATA1<<8) | u8_ADCDATA0;
//	Serial.println("Have data word"); //debug
	return uiADCDATA;
}

float CN0357class::SetParameters(float max_sens, float sens_range, float VREF)
{
  float fFeedback = 0;

  fFeedback = VREF /(sens_range*max_sens*pow(10,-9)); //compute the necessary feedback resistance based on sensor parameters
  return fFeedback;
  //uiRDAC = CN0357.Set_Rheostat(fFeedback, Rheostat_ss); // set feedback resistance, returns approximate value to the nearest LSB
}


float CN0357class::Set_Rheostat(float resistance, float Rheostat_fs, float Rheostat_res, byte Rheostat_ss) //set feedback resistance for sensor current to voltage conversion
{
	unsigned long ulRDAC_code;
	byte RDAC_lo = 0;
	byte RDAC_hi = 0; 
	float resistorValue = 0;
	
	ulRDAC_code = (resistance/(Rheostat_fs/Rheostat_res));//compute RDAC data to be written 

	RDAC_hi = byte(ulRDAC_code>>8);
	RDAC_lo = byte(ulRDAC_code & 0xFF);
	
	digitalWrite(Rheostat_ss,LOW);	//Write RDAC value to rheostat
	SPI.transfer(RDAC_hi | 0x04);	
	SPI.transfer(RDAC_lo);	
	digitalWrite(Rheostat_ss,HIGH);
	Rheostat_SDO_HiZ(Rheostat_ss);
	
	resistorValue = ulRDAC_code*(Rheostat_fs/Rheostat_res);
	return (resistorValue);	//return theoretical resistance written to the RDAC
}


float CN0357class::ReadPPM(float typ_sens, float VREF, float resistorValue, byte ADC_ss)
{
float fADCvoltage = 0;
float fConcentration = 0;
unsigned int uiADCvoltage = 0;
 // Serial.println("Start Read"); //debug
  uiADCvoltage = CN0357.Readdata(ADC_ss); //read ADC code
  fADCvoltage = ((uiADCvoltage/pow(2,15))-1)*VREF; //compute for ADC voltage input
  fConcentration = (abs(fADCvoltage) / resistorValue) / (typ_sens*pow(10,-9)); //compute gas concentration
 // Serial.println("Finish Read"); //debug
  return fConcentration;
}

