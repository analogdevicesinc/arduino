/*
  CN0357_example.ino - Example code for CN0357 - Toxic Gas Detector
 Created by Analog Devices Inc. - Circuits from the Lab, December 2014.
 */

#include <Arduino.h>
#include <SPI.h>
#include "AD5270.h"
#include "AD7790.h"

// AD5270 Pin Assignments
#define AD5270_SS      6  // value of the CS pin assignment

// AD7790 Pin Assignments
#define AD7790_SS      8  // value of the CS pin assignment

//  Gas Sensor Variables
uint16_t ui16sensorRange = 2000;			//value is in units (PPM)
uint16_t ui16sensitivity =  65;			//value is in units (nA/ppm)

// AD5270 variables
float fResistorValue = 0;
uint16_t ui16RdacWord = 0;

// AD7790 variables
uint16_t ui16Adcdata = 0;

// Main variables
float fAdcVoltage = 0;
float fConcentration = 0;

void setup() {
	
	// open digital communication protocols
	Serial.begin(9600);
	SPI.begin();

        // initialize pins
	pinMode(AD5270_SS,OUTPUT);	        //set CS pin for sensor
	digitalWrite(AD5270_SS,HIGH);		//brings chip select high 
    
	pinMode(AD7790_SS,OUTPUT);	        //set CS pin for LCD
	digitalWrite(AD7790_SS,HIGH);		//brings chip select high 
    
        // initialize AD5270
        AD5270.AD5270_SPI_Configuration();
	Ad5270INIT();
	
	// set digipot value
	fResistorValue = calculateFeedbackResistor();
	ui16RdacWord = setResistorValue(fResistorValue);
	AD5270.writeAd5270 (WRITE_RDAC, ui16RdacWord);
        AD5270.writeAd5270 (HI_Z_PREP, 0x8001);  // Putting Rheostat into high Z mode on SDO line
        AD5270.writeAd5270 (HI_Z, 0x0000);
   	Serial.print("Calculated Digitpot Value = ");
	Serial.println(fResistorValue); 
	Serial.print("Actual Digitpot Value = ");
	Serial.println(ui16RdacWord); 

	// initialize AD7790
	AD7790.AD7790_SPI_Configuration();
	Ad7790INIT();	
}

void loop() {

	do
        {
          ui16Adcdata = AD7790.readAd7790(STATUS_READ);
          Serial.print("ADC Status Reg value = ");
	  Serial.println(ui16Adcdata); 
        }while (ui16Adcdata & 0x80); 

        ui16Adcdata = AD7790.readAd7790(DATA_READ);
	fAdcVoltage = ((ui16Adcdata / pow(2,15))-1)*1.2;		// Formula for input voltage using bipolar configuration
	fConcentration = (abs(fAdcVoltage)/ (ui16RdacWord*(20000/1024))) / (ui16sensitivity*pow(10,-9));
	Serial.print("ADC Data Reg value = ");
        Serial.println(ui16Adcdata);
        Serial.print("Sensor Voltage value = ");
	Serial.println(fAdcVoltage);

        Serial.print("Carbon Monoxide (CO) concentration = ");
	Serial.print(fConcentration,2);  //display gas concentration
	Serial.println(" PPM");
	delay (1000);
}

void Ad7790INIT(void)
{
	AD7790.writeAd7790 (RESET, 0xFF);				//Resets the part for initial use
	delay(1000);
	AD7790.writeAd7790 (MODE_WRITE, 0x00);			//Mode register value (single conversion, +/- Vref input, unbuffered mode)
	AD7790.writeAd7790 (FILTER_WRITE, 0x07);		// Filter register value (clock not divided down, 9.5 Hz update rate)
}

void Ad5270INIT(void)
{
	AD5270.writeAd5270 (WRITE_CTRL_REG, 0x02);		//Enable RDAC writes
}

float calculateFeedbackResistor(void)
{
	float fFeedback = 0;
	fFeedback = 1.2 / (ui16sensorRange * ui16sensitivity * pow(10,-9));			//1.2 is the Vref of the circuit
	return fFeedback;
}

uint16_t setResistorValue(float resistor)
{
	uint16_t ui16RdacCode = 0;
	ui16RdacCode = int(resistor / (20000/1024));
	return ui16RdacCode;
}
