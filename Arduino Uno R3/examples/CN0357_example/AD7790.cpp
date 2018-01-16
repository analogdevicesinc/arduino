/*
  AD7790.cpp - 
*/

#include <Arduino.h>
#include <SPI.h>
#include "AD7790.h"

AD7790class AD7790;

void AD7790class::AD7790_SPI_Configuration(void)
{
	SPI.setBitOrder(MSBFIRST);          		//  MSB to be sent first
	SPI.setDataMode(SPI_MODE3);         		//  Set for clock rising edge, clock idles low
	SPI.setClockDivider(SPI_CLOCK_DIV128);		//  Set clock divider (optional)
	delay(100);
}

uint16_t AD7790class::readAd7790 (uint8_t ui8address)
{
	uint8_t ui8AdcUpperCodes = 0;			// Data register read MSB
	uint8_t ui8AdcLowerCodes = 0;			// Data register read LSB
	uint16_t ui16AdcCodes = 0;

	if (ui8address == DATA_READ)
	{	
		digitalWrite(AD7790_SS,LOW);
		SPI.transfer(ui8address); 
		ui8AdcUpperCodes = SPI.transfer(0x00);			//Data register read MSB
		ui8AdcLowerCodes = SPI.transfer(0x00);			//Data register read LSB
		digitalWrite(AD7790_SS,HIGH);
		ui16AdcCodes = ((long)ui8AdcUpperCodes << 8) | ui8AdcLowerCodes;
		
		/*Serial.print("ADC Data Register Read : ");  //Debug serial prints
		Serial.println(ui16AdcCodes,2);*/
		
		/*Serial.print("ADC Upper 8 Bits : ");		//Debug serial prints
		Serial.println(ui8AdcUpperCodes,2);
		Serial.print("ADC Lower 8 Bits : ");  
		Serial.println(ui8AdcLowerCodes,2);*/
	}
	else
	{
		digitalWrite(AD7790_SS,LOW);
		SPI.transfer(ui8address); 
		ui8AdcLowerCodes = SPI.transfer(0x00);			// register read
		digitalWrite(AD7790_SS,HIGH);	
		ui16AdcCodes = ((long)ui8AdcUpperCodes << 8) | ui8AdcLowerCodes;
		
		/*Serial.print("ADC Register Being Read: ");		//Debug serial prints
		Serial.println(ui8address);
		Serial.print(" Reading Register Value : ");
		Serial.println(ui16AdcCodes);*/
	}

	return ui16AdcCodes;
}

void AD7790class::writeAd7790 (uint8_t ui8address, uint8_t ui8value)
{
	
	if (ui8address != RESET)
	{
		digitalWrite(AD7790_SS,LOW);
		SPI.transfer(ui8address); 
		SPI.transfer(ui8value);
		digitalWrite(AD7790_SS,HIGH);
		//Serial.println("Write Command");		//Debug serial prints
	}
	else
	{
		digitalWrite(AD7790_SS,LOW);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		SPI.transfer(ui8value);
		digitalWrite(AD7790_SS,HIGH);
		//Serial.println("Reset Command");			//Debug serial prints
	}
	
	/*Serial.print("ADC Register ");		//Debug serial prints
	Serial.print(ui8address);
	Serial.println("Written");
	Serial.print("With Register Value ");
	Serial.println(ui8value);*/
}
