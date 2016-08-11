/* AD5270.c
 *
 *  Created on: May 27, 2015
 *      Author: BBUSHEY
 */

#include <Arduino.h>
#include <SPI.h>
#include "AD5270.h"

AD5270class AD5270;

void AD5270class::AD5270_SPI_Configuration(void)
{
	SPI.setBitOrder(MSBFIRST);          		//  MSB to be sent first
	SPI.setDataMode(SPI_MODE3);         		//  Set for clock rising edge, clock idles low
	SPI.setClockDivider(SPI_CLOCK_DIV128);		//  Set clock divider (optional)
	delay(100);
}

void AD5270class::writeAd5270 (uint8_t ui8command, uint16_t ui16value)
{
	uint8_t ui8DacUpperCodes = 0;			// Data register read MSB
	uint8_t ui8DacLowerCodes = 0;			// Data register read MSB
	uint16_t ui16DacCodes = 0;

	if (ui8command == 1)
	{
		ui16DacCodes = (ui8command << 10);
		ui16DacCodes = (ui16DacCodes | (ui16value));
		ui8DacLowerCodes = ui16DacCodes & 0xFF;
		ui8DacUpperCodes = (ui16DacCodes >> 8) & 0xFF;
	}
	if (ui8command == 5 || ui8command == 7 || ui8command == 9)
	{
		ui8DacLowerCodes = ui16value & 0xFF;
		ui8DacUpperCodes = (ui8command << 2);
	}
	if (ui8command == 0xF0)			//SDO High Z prep
	{
		ui8DacLowerCodes = ui16value & 0xFF;
		ui8DacUpperCodes = (ui16value >> 8) & 0xFF;
	}
	if (ui8command == 0x0F)			// SDO High Z
	{
		ui8DacLowerCodes = ui16value & 0xFF;
		ui8DacUpperCodes = (ui16value >> 8) & 0xFF;
	}
	digitalWrite(AD5270_SS,LOW);
	SPI.transfer(ui8DacUpperCodes); 
	SPI.transfer(ui8DacLowerCodes); 
	digitalWrite(AD5270_SS,HIGH);
	
	/*Serial.print("Digipot Resistor Code = ");			//Debug serial prints
    Serial.println(ui16DacCodes);
	Serial.print("Digipot Upper 8 Bits = ");
    Serial.println(ui8DacUpperCodes);
	Serial.print("Digipot Lower 8 Bits = ");
    Serial.println(ui8DacLowerCodes);*/
}

uint16_t AD5270class::readAd5270 (uint8_t ui8command)
{
	uint8_t ui8DacUpperCodes = 0;			// Data register read MSB
	uint8_t ui8DacLowerCodes = 0;			// Data register read LSB
	uint8_t ui8DacReadUpper = 0;
	uint8_t ui8DacReadLower = 0;
	uint16_t ui16DacRead = 0;

	if ((ui8command != 1) || (ui8command != 5) || (ui8command != 7) || (ui8command != 9))
	{
		ui8DacLowerCodes = 0;
		ui8DacUpperCodes = (ui8command << 2);
		
		digitalWrite(AD5270_SS,LOW);
		SPI.transfer(ui8DacUpperCodes); 
		SPI.transfer(ui8DacLowerCodes); 
		ui8DacReadUpper = SPI.transfer(0x00);			//Data register read MSB
		ui8DacReadLower = SPI.transfer(0x00);			//Data register read LSB
		digitalWrite(AD5270_SS,HIGH);
		
		ui16DacRead = ((ui8DacReadUpper << 8) | ui8DacReadLower) & 0x03FF;

	}
	
	return ui16DacRead;
}