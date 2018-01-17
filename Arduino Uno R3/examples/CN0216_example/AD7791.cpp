/*
  AD7791.cpp - 
*/

#include <Arduino.h>
#include <SPI.h>
#include "AD7791.h"

AD7791class AD7791;

void AD7791class::AD7791_SPI_Configuration(void)
{
	SPI.setBitOrder(MSBFIRST);          		//  MSB to be sent first
	SPI.setDataMode(SPI_MODE3);         		//  Set for clock rising edge, clock idles low
	SPI.setClockDivider(SPI_CLOCK_DIV128);		//  Set clock divider (optional)
	delay(100);
}

uint32_t AD7791class::readAd7791 (uint8_t ui8address)
{
	uint8_t ui8AdcUpperCodes = 0;			// Data register bits 23-16
	uint8_t ui8AdcMiddleCodes = 0;			// Data register bits 15-8
	uint8_t ui8AdcLowerCodes = 0;			// Data register bits 7-0
	uint32_t ui32AdcCodes = 0;

	if (ui8address == DATA_READ)
	{	
		digitalWrite(AD7791_SS,LOW);
		SPI.transfer(ui8address); 
		ui8AdcUpperCodes = SPI.transfer(0x00);			// Data register bits 23-16
		ui8AdcMiddleCodes = SPI.transfer(0x00);			// Data register bits 15-8
		ui8AdcLowerCodes = SPI.transfer(0x00);			// Data register bits 7-0
		digitalWrite(AD7791_SS,HIGH);
		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;
		
		/*Serial.print("ADC Data Register Read : ");  //Debug serial prints
		Serial.println(ui32AdcCodes,2);			
		Serial.print("ADC Bits 23-16 : ");		
		Serial.println(ui8AdcUpperCodes,2);
		Serial.print("ADC Bits 15-8 : ");  
		Serial.println(ui8AdcMiddleCodes,2);
		Serial.print("ADC Bits 7-0 : ");  
		Serial.println(ui8AdcLowerCodes,2);*/
	}
	else
	{
		digitalWrite(AD7791_SS,LOW);
		SPI.transfer(ui8address); 
		ui8AdcLowerCodes = SPI.transfer(0x00);			// register read
		digitalWrite(AD7791_SS,HIGH);	
		ui32AdcCodes = ((long)ui8AdcUpperCodes << 16) | ((long)ui8AdcMiddleCodes << 8) | ui8AdcLowerCodes;
		
		/*Serial.print("ADC Register Being Read:");		//Debug serial prints
		Serial.println(ui8address);
		Serial.print(" Reading Register Value :");
		Serial.println(ui32AdcCodes);*/
	}

	return ui32AdcCodes;
}

void AD7791class::writeAd7791 (uint8_t ui8address, uint8_t ui8value)
{
	
	if (ui8address != RESET)
	{
		digitalWrite(AD7791_SS,LOW);
		SPI.transfer(ui8address); 
		SPI.transfer(ui8value);
		digitalWrite(AD7791_SS,HIGH);
		//Serial.println("Write Command");		//Debug serial prints
	}
	else
	{
		digitalWrite(AD7791_SS,LOW);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		digitalWrite(AD7791_SS,HIGH);
		//Serial.println("Reset Command");			//Debug serial prints
	}
	
	/*Serial.print("ADC Register ");		//Debug serial prints
	Serial.print(ui8address);
	Serial.println("Written");
	Serial.print("With Register Value ");
	Serial.println(ui8value);*/
}