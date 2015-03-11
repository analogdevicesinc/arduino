/*
 CN0357_example.ino - Example code for CN0357 - Toxic Gas Detector
 Created by Analog Devices Inc. - Circuits from the Lab, March 2015.
 */
#include <CN0357.h>
#include <SPI.h>

/*
SS pin settings for ADC
Resistor to be Populated      GPIO Pin
R15(default)                     8
R16                              9
R17                              10
*/
const int CN0357_ADC_SS = 8;  //change the value if the SS pin assignment for ADC is changed

/*
SS pin settings for Rheostat
Resistor to be Populated      GPIO Pin
R19(default)                     6
R20                              7
*/
const int CN0357_RHEOSTAT_SS = 6; //change the value if the SS pin assignment for rheostat is changed

// ADC Register Settings
const byte CN0357_ADCMODE_UNBUF = 0x00;
const byte CN0357_ADCMODE_BUF = 0x02;

const byte CN0357_ADCRATE_9_5  = 7;
const byte CN0357_ADCRATE_13_3 = 6;
const byte CN0357_ADCRATE_16_6 = 5;
const byte CN0357_ADCRATE_16_7 = 4;
const byte CN0357_ADCRATE_20 = 3;
const byte CN0357_ADCRATE_33_3 = 2;
const byte CN0357_ADCRATE_100 = 1;
const byte CN0357_ADCRATE_120 = 0;

//sensor parameters
const int MAX_SENSITIVITY = 100; //enter maximum sensor sensitivity in nA/ppm
const int TYP_SENSITIVITY = 65;  //enter typical sensor sensitivity in nA/ppm
const int RANGE = 2000;  //enter maximum sensor range in ppm

//Rheostat parameters
const float RHEOSTAT_FS = 20000.0; //Full scale rheostat resistance in ohms
const float RHEOSTAT_RES = 1024; //Number of codes in rheostat

//Sketch variables
const float VREF = 1.2;
int fRDAC = 0;
float fGasPPM = 0;
float fCalculatedResistance = 0;
float fActualResistance = 0;

void setup() {

  Serial.begin(9600); //sets the baud rate for UART/Serial communication
  CN0357.Configure(CN0357_ADCMODE_UNBUF,CN0357_ADCRATE_9_5,CN0357_ADC_SS,CN0357_RHEOSTAT_SS); //set ADC mode and data rate. set to unbuffered mode at 9.5Hz
  fCalculatedResistance = CN0357.SetParameters(MAX_SENSITIVITY,RANGE,VREF); //configures system based on sensor parameters
  fActualResistance = CN0357.Set_Rheostat(fCalculatedResistance,RHEOSTAT_FS,RHEOSTAT_RES,CN0357_RHEOSTAT_SS);

}

void loop() {
  
  delay(1000);
  fGasPPM = CN0357.ReadPPM(TYP_SENSITIVITY,VREF,fActualResistance,CN0357_ADC_SS); //read data from system and computes gas concentration(ppm)
  Serial.print(fGasPPM,2);  //display gas concentration
  Serial.println(" PPM");  

}

