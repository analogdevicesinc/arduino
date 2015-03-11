/*
  CN0357.h - Library for CN0357 - Universal Toxic Gas Sensor
  Created by Analog Devices Inc. - Circuits from the Lab, December 2014.
*/
#ifndef CN0357_h
#define CN0357_h

#include "Arduino.h"

const byte byte_zero = 0x00;
const byte byte_full = 0xFF;

class CN0357class
{
	public:
		void Configure(byte ADCmode,byte ADCrate,byte ADC_ss, byte Rheostat_ss);
		unsigned int Readdata(byte ADC_ss);
		void Rheostat_SDO_HiZ(byte Rheostat_ss);
		float Set_Rheostat(float resistance, float Rheostat_fs, float Rheostat_res, byte Rheostat_ss);
		float SetParameters(float max_sens, float sens_range, float VREF);
		float ReadPPM(float typ_sens, float VREF, float resistorValue, byte ADC_ss);
		//float fRDAC;
	private:
		volatile byte cRDY;	
		byte u8_ADCDATA0;
	    byte u8_ADCDATA1;
	    unsigned int uiADCDATA;
};

extern CN0357class CN0357;

#endif
