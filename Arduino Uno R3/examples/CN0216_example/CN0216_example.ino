/*
 CN0216_example.ino - Example code for CN0216 - Weigh Scale
 Created by Analog Devices Inc. - Circuits from the Lab, June 2015.
 */

#include <Arduino.h>
#include <SPI.h>
#include "AD7791.h"

// AD7790 Pin Assignme nts
#define AD7791_SS      8  // value of the CS pin assignment

//  Weigh Scale Variables
uint16_t ui16calibrationWeight = 1000;	  //value is in units (grams)

// AD7791 variables
uint32_t ui32Adcdata = 0;

// Main program variables
float fgramsPerCode = 0.0;
float fWeight = 0.0;
float fzeroScaleCalibration = 0.0;
float ffullScaleCalibration = 0.0;

void setup() {
	
	// open digital communication protocols
	Serial.begin(9600);
	SPI.begin();

        // initialize pins
	pinMode(AD7791_SS,OUTPUT);	        //set CS pin
	digitalWrite(AD7791_SS,HIGH);		//brings chip select high 
	
	// initialize AD7791
	AD7791.AD7791_SPI_Configuration();
	Ad7791INIT();

        // calibrating the scale	
        Serial.println("Zero Scale Calibration in process, please wait 15 seconds ");
        fzeroScaleCalibration = getCalibrationData();
        Serial.println("Zero Scale Calibration Complete ");
        delay(1000);
        Serial.println("Please put Calibration Weight on the Scale ");
        delay(8000);
        Serial.println("Full Scale Calibration in process, please wait 15 seconds ");
        ffullScaleCalibration = getCalibrationData();
        Serial.println("Full Scale Calibration Complete ");
        Serial.println("Calibration Process Complete ");  
        delay(3000);      
}

void loop() {

	do
        {
          ui32Adcdata = AD7791.readAd7791(STATUS_READ);
          //Serial.print("ADC Status Reg value = ");  //Debug serial prints
	  //Serial.println(ui32Adcdata); 
        }while (ui32Adcdata & 0x80); 

        ui32Adcdata = AD7791.readAd7791(DATA_READ);
	fgramsPerCode = ui16calibrationWeight / (ffullScaleCalibration - fzeroScaleCalibration);
        fWeight = (float(ui32Adcdata) - fzeroScaleCalibration) * fgramsPerCode;
        
        /*Serial.print("Zero Calibration Reading = ");  //Debug serial prints
	Serial.println(fzeroScaleCalibration);  //display zero scale calibration data
        Serial.print("Full Calibration Reading = ");
	Serial.println(ffullScaleCalibration);  //display full scale calibration data
        Serial.print("ADC Data Register Code = ");
	Serial.println(ui32Adcdata);  //display data register value
        Serial.print("Grams per code = ");
	Serial.println(fgramsPerCode);  //display weight on scale*/
       
       
        Serial.print("Weigh Scale Value = ");
	Serial.print(fWeight,2);  //display weight on scale
	Serial.println(" grams");
	delay (1000);
}

void Ad7791INIT(void)
{
	AD7791.writeAd7791 (RESET, 0xFF);		//Resets the part for initial use
	delay(1000);
	AD7791.writeAd7791 (MODE_WRITE, 0x00);	        //Mode register value (single conversion, +/- Vref input, unbuffered mode)
	AD7791.writeAd7791 (FILTER_WRITE, 0x07);	// Filter register value (clock not divided down, 9.5 Hz update rate)
}

float getCalibrationData(void)
{
    uint32_t ui32calibrationData = 0;
    uint32_t ui32status = 0;
    uint8_t x = 0;
    float fcalibration = 0.0;
    
    for (x = 0; x < 100; x++)
    { 
        ui32status = 0;
        do
        {
          ui32status = AD7791.readAd7791(STATUS_READ);
          //Serial.print("ADC Status Reg value = ");  //Debug serial prints
	  //Serial.println(ui32Adcdata); 
        }while (ui32status & 0x80); 
        
        ui32calibrationData += AD7791.readAd7791(DATA_READ);
        delay(100);
    }
        fcalibration = ui32calibrationData/100.0;
	return fcalibration;
}
