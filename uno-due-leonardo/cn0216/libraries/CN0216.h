/*
  CN0216.h - Library for CN0216 Embedded World Demo- Weighing Scale Module
  Created by Analog Devices Inc. - Circuits from the Lab, January 2015.
*/
#ifndef CN0216_h
#define CN0216_h

#include "Arduino.h"
const byte byte_zero = 0x00;
const byte byte_full = 0xFF;
const float VREF = 5.0/16777216;

class CN0216class
{
	public:
		void Configure(byte ADCmode,byte ADCrate,byte CN0216_ss_pin);
		long Readdata();
		void Calibrate_zero();
		void Calibrate_fullscale();
		float Readweight(int fcal_wt_input);
	private:
		unsigned long ulADCZEROSCALE;
		unsigned long ulADCFULLSCALE;
		float fADCZEROSCALE;
		float fADCFULLSCALE;
		volatile byte cRDY;	
		byte u8_ADCDATA0;
	    byte u8_ADCDATA1;
	    byte u8_ADCDATA2;
	    long lADCDATA;
		byte ss_pin;
};

extern CN0216class CN0216;

#endif
