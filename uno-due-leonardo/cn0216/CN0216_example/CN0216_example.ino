/*
 CN0216_example.ino - Example code for CN0216 - Weighing Scale Module
 Created by Analog Devices Inc. - Circuits from the Lab, March 2015.
*/

#include <CN0216.h>
#include <SPI.h> 

// ADC mode register selections
const int CN0216_ADCMODE_UNBUF = 0x00;
const int CN0216_ADCMODE_BUF = 0x02;

// ADC filter register selections
const int CN0216_ADCRATE_9_5 = 7;
const int CN0216_ADCRATE_13_3 = 6; 
const int CN0216_ADCRATE_16_6 = 5; 
const int CN0216_ADCRATE_16_7 = 4; 
const int CN0216_ADCRATE_20 = 3; 
const int CN0216_ADCRATE_33_3 = 2; 
const int CN0216_ADCRATE_100 = 1; 
const int CN0216_ADCRATE_120 = 0; 

// SPI chip select 
const int CN0216_SS = 8;  //change the value if the CS pin assignment is different 

// Weigh Scale parameters
const int CAL_WT = 1000;  //value of calibration weight used, in grams

// Sketch variables
float fWeight = 0;

void setup() {
  // put your setup code here, to run once:
  CN0216.Configure(CN0216_ADCMODE_UNBUF,CN0216_ADCRATE_9_5,CN0216_SS);
  Serial.begin(9600);

  //Calibration for zero scale
  Serial.println("Zero calibration will start in 5 seconds, please remove weight from the sensor\n");
  delay(5000);
  Serial.println("Zero calibration in process please wait approximately 20 seconds\n");
  CN0216.Calibrate_zero();
  Serial.println("Zero calibration finished\n");
  delay(1000);
  
  //Full scale calibration routine
  Serial.println("Full scale calibration will start in 10 seconds, please place weight on sensor\n");
  delay(10000);
  Serial.println("Full scale calibration in process please wait approximately 20 seconds\n");
  CN0216.Calibrate_fullscale();
  Serial.println("Calibration finished\n");
  delay(1000);  
}

void loop() {
  fWeight = CN0216.Readweight(CAL_WT);
  Serial.print(fWeight,2);  
  Serial.println(" grams");
  delay(1000);
}

