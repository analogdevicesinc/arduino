/**************************************************************************
  ADE9153AAPI.cpp - Library for ADE9153A energy measurement IC with mSure 
  autocalibration
  
  Designed specifically to work with the EV_ADE9153ASHIELDZ board
  ---- http://www.analog.com/ADE9153A
  
  Created by David Lath for Analog Devices Inc., January 8, 2018
  
  Copyright (c) 2018, Analog Devices, Inc.  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted (subject to the limitations in the disclaimer 
  below) provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright 
  notice, this list of conditions and the following disclaimer in the 
  documentation and/or other materials provided with the distribution.

  * Neither the name of Analog Devices, Inc. nor the names of its 
  contributors may be used to endorse or promote products derived from this 
  software without specific prior written permission.

  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED 
  BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
  BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include "ADE9153AAPI.h"
#include <Wire.h>

ADE9153AClass::ADE9153AClass()
{
	
}

/* 
Description: Initializes the ADE9153A. The initial settings for registers are defined in ADE9153AAPI.h header file
Input: Register settings in header files
Output:- 
*/

void ADE9153AClass::SetupADE9153A(void)
{
	SPI_Write_16(REG_AI_PGAGAIN,ADE9153A_AI_PGAGAIN);     
 	SPI_Write_32(REG_CONFIG0,ADE9153A_CONFIG0); 
	SPI_Write_16(REG_CONFIG1,ADE9153A_CONFIG1);
	SPI_Write_16(REG_CONFIG2,ADE9153A_CONFIG2);
	SPI_Write_16(REG_CONFIG3,ADE9153A_CONFIG3);
	SPI_Write_16(REG_ACCMODE,ADE9153A_ACCMODE);
	SPI_Write_32(REG_VLEVEL,ADE9153A_VLEVEL);
	SPI_Write_16(REG_ZX_CFG,ADE9153A_ZX_CFG);
	SPI_Write_32(REG_MASK,ADE9153A_MASK);
	SPI_Write_32(REG_ACT_NL_LVL,ADE9153A_ACT_NL_LVL);
	SPI_Write_32(REG_REACT_NL_LVL,ADE9153A_REACT_NL_LVL);
	SPI_Write_32(REG_APP_NL_LVL,ADE9153A_APP_NL_LVL);
	SPI_Write_16(REG_COMPMODE,ADE9153A_COMPMODE);
	SPI_Write_32(REG_VDIV_RSMALL,ADE9153A_VDIV_RSMALL);
	SPI_Write_16(REG_EP_CFG,ADE9153A_EP_CFG);
	SPI_Write_16(REG_EGY_TIME,ADE9153A_EGY_TIME);		//Energy accumulation ON
	SPI_Write_16(REG_TEMP_CFG,ADE9153A_TEMP_CFG);
}

/* 
Description: Initializes the arduino SPI port using SPI.h library
Input: SPI speed, chip select pin
Output:-
*/

bool ADE9153AClass::SPI_Init(uint32_t SPI_speed , uint8_t chipSelect_Pin)
{
	digitalWrite(chipSelect_Pin, HIGH);		//Set Chip select pin high 
	pinMode(chipSelect_Pin, OUTPUT);		//Set Chip select pin as output	
	SPI.begin();		//Initiate SPI port
	SPI.beginTransaction(SPISettings(SPI_speed,MSBFIRST,SPI_MODE0));		//Setup SPI parameters

	_chipSelect_Pin = chipSelect_Pin;
	
	SPI_Write_16(REG_RUN,ADE9153A_RUN_ON);
	delay(100);
	if (SPI_Read_32(REG_VERSION_PRODUCT) != 0x0009153A)
		return false;
	
	return true;
}

/* 
Description: Writes 16bit data to a 16 bit register. 
Input: Register address, data
Output:-
*/

void ADE9153AClass:: SPI_Write_16(uint16_t Address , uint16_t Data )
{
	uint16_t temp_address;
	
	digitalWrite(_chipSelect_Pin, LOW);
	temp_address = ((Address << 4) & 0xFFF0);	//shift address  to align with cmd packet
	SPI.transfer16(temp_address);
	SPI.transfer16(Data);
	
	digitalWrite(_chipSelect_Pin, HIGH); 	
}

/* 
Description: Writes 32bit data to a 32 bit register. 
Input: Register address, data
Output:-
*/

void ADE9153AClass:: SPI_Write_32(uint16_t Address , uint32_t Data )
{
	uint16_t temp_address;
	uint16_t temp_highpacket;
	uint16_t temp_lowpacket;

	temp_highpacket= (Data & 0xFFFF0000)>>16;
	temp_lowpacket= (Data & 0x0000FFFF);
	
	digitalWrite(_chipSelect_Pin, LOW);
	
	temp_address = ((Address << 4) & 0xFFF0);	//shift address  to align with cmd packet
	SPI.transfer16(temp_address);
	SPI.transfer16(temp_highpacket);
	SPI.transfer16(temp_lowpacket);
	
	digitalWrite(_chipSelect_Pin, HIGH); 	
	
}

/* 
Description: Reads 16bit data from register. 
Input: Register address
Output: 16 bit data
*/

uint16_t ADE9153AClass:: SPI_Read_16(uint16_t Address)
{
	uint16_t temp_address;
	uint16_t returnData;
	
	digitalWrite(_chipSelect_Pin, LOW);
	
	temp_address = (((Address << 4) & 0xFFF0)+8);
	SPI.transfer16(temp_address);
	returnData = SPI.transfer16(0);
	
	digitalWrite(_chipSelect_Pin, HIGH);
	return returnData;
}

/* 
Description: Reads 32bit data from register. 
Input: Register address
Output: 32 bit data
*/

uint32_t ADE9153AClass:: SPI_Read_32(uint16_t Address)
{
	uint16_t temp_address;
	uint32_t temp_highpacket;
	uint16_t temp_lowpacket;
	uint32_t returnData;
	
	digitalWrite(_chipSelect_Pin, LOW);
	
	temp_address = (((Address << 4) & 0xFFF0)+8);
	SPI.transfer16(temp_address);
	temp_highpacket = SPI.transfer16(0);
	temp_lowpacket = SPI.transfer16(0);	
	
	digitalWrite(_chipSelect_Pin, HIGH);
	
	returnData = temp_highpacket << 16;
	returnData = returnData + temp_lowpacket;
	
	return returnData;
}

/* 
Description: Reads the metrology data from the ADE9153A
Input: Structure name
Output: Respective metrology data
*/

void ADE9153AClass:: ReadEnergyRegs(EnergyRegs *Data)
{
	int32_t tempReg;
	float tempValue;
	
	tempReg = int32_t (SPI_Read_32(REG_AWATTHR_HI));
	Data->ActiveEnergyReg = tempReg;
	tempValue = (float)tempReg * CAL_ENERGY_CC / 1000;
	Data->ActiveEnergyValue = tempValue;				//Energy in mWhr
	
	tempReg = int32_t (SPI_Read_32(REG_AFVARHR_HI));
	Data->FundReactiveEnergyReg = tempReg;
	tempValue = (float)tempReg * CAL_ENERGY_CC / 1000;
	Data->FundReactiveEnergyValue = tempValue;			//Energy in mVARhr
	
	tempReg = int32_t (SPI_Read_32(REG_AVAHR_HI));
	Data->ApparentEnergyReg = tempReg;
	tempValue = (float)tempReg * CAL_ENERGY_CC / 1000;
	Data->ApparentEnergyValue = tempValue;				//Energy in mVAhr
}

void ADE9153AClass:: ReadPowerRegs(PowerRegs *Data)
{
	int32_t tempReg;
	float tempValue;
	
	tempReg = int32_t (SPI_Read_32(REG_AWATT));
	Data->ActivePowerReg = tempReg;
	tempValue = (float)tempReg * CAL_POWER_CC / 1000;
	Data->ActivePowerValue = tempValue;					//Power in mW
	
	tempReg = int32_t (SPI_Read_32(REG_AFVAR));
	Data->FundReactivePowerReg = tempReg;
	tempValue = (float)tempReg * CAL_POWER_CC / 1000;
	Data->FundReactivePowerValue = tempValue;			//Power in mVAR
	
	tempReg = int32_t (SPI_Read_32(REG_AVA));
	Data->ApparentPowerReg = tempReg;
	tempValue = (float)tempReg * CAL_POWER_CC / 1000;
	Data->ApparentPowerValue = tempValue;				//Power in mVA
}

void ADE9153AClass:: ReadRMSRegs(RMSRegs *Data)
{
	uint32_t tempReg;
	float tempValue;
	
	tempReg = int32_t (SPI_Read_32(REG_AIRMS));
	Data->CurrentRMSReg = tempReg;
	tempValue = (float)tempReg * CAL_IRMS_CC / 1000;	//RMS in mA
	Data->CurrentRMSValue = tempValue;
	
	tempReg = int32_t (SPI_Read_32(REG_AVRMS));
	Data->VoltageRMSReg = tempReg;
	tempValue = (float)tempReg * CAL_VRMS_CC / 1000;	//RMS in mV
	Data->VoltageRMSValue = tempValue;
}

void ADE9153AClass:: ReadHalfRMSRegs(HalfRMSRegs *Data)
{
	uint32_t tempReg;
	float tempValue;
	
	tempReg = int32_t (SPI_Read_32(REG_AIRMS_OC));
	Data->HalfCurrentRMSReg = tempReg;
	tempValue = (float)tempReg * CAL_IRMS_CC / 1000;	//Half-RMS in mA
	Data->HalfCurrentRMSValue = tempValue;
	
	tempReg = int32_t (SPI_Read_32(REG_AVRMS_OC));
	Data->HalfVoltageRMSReg = tempReg;
	tempValue = (float)tempReg * CAL_VRMS_CC / 1000;	//Half-RMS in mV
	Data->HalfVoltageRMSValue = tempValue;
}

void ADE9153AClass:: ReadPQRegs(PQRegs *Data)
{
	int32_t tempReg;
	uint16_t temp;
	float mulConstant;
	float tempValue;
	
	tempReg=int32_t (SPI_Read_32(REG_APF)); //Read PF register
	Data->PowerFactorReg = tempReg;
	tempValue=(float)tempReg/(float)134217728; //Calculate PF
	Data->PowerFactorValue=tempValue;
	
	tempReg=int32_t (SPI_Read_32(REG_APERIOD)); //Read PERIOD register
	Data->PeriodReg = tempReg;
	tempValue=(float)(4000*65536)/(float)(tempReg+1); //Calculate Frequency
	Data->FrequencyValue = tempValue;
	
	temp=SPI_Read_16(REG_ACCMODE); //Read frequency setting register
	if((temp&0x0010)>0)
		{
			mulConstant=0.02109375;  //multiplier constant for 60Hz system
		}
	else
		{
			mulConstant=0.017578125; //multiplier constant for 50Hz system		
		}
	
	tempReg=int16_t (SPI_Read_16(REG_ANGL_AV_AI)); //Read ANGLE register
	Data->AngleReg_AV_AI=tempReg;
	tempValue=tempReg*mulConstant;	//Calculate Angle in degrees
	Data->AngleValue_AV_AI=tempValue;
}

void ADE9153AClass:: ReadAcalRegs(AcalRegs *Data)
{
	uint32_t tempReg;
	float tempValue;
	
	tempReg=int32_t (SPI_Read_32(REG_MS_ACAL_AICC)); //Read AICC register
	Data->AcalAICCReg = tempReg;
	tempValue=(float)tempReg/(float)2048; //Calculate Conversion Constant (CC)
	Data->AICC=tempValue;
	tempReg=int32_t (SPI_Read_32(REG_MS_ACAL_AICERT)); //Read AICERT register
	Data->AcalAICERTReg = tempReg;
	
	tempReg=int32_t (SPI_Read_32(REG_MS_ACAL_AVCC)); //Read AVCC register
	Data->AcalAVCCReg = tempReg;
	tempValue=(float)tempReg/(float)2048; //Calculate Conversion Constant (CC)
	Data->AVCC=tempValue;
	tempReg=int32_t (SPI_Read_32(REG_MS_ACAL_AVCERT)); //Read AICERT register
	Data->AcalAVCERTReg = tempReg;
}

/* 
Description: Start autocalibration on the respective channel
Input: -
Output: Did it start correctly?
*/

bool ADE9153AClass::StartAcal_AINormal(void)
{
	uint32_t ready;
	int waitTime = 0;
	
	ready = SPI_Read_32(REG_MS_STATUS_CURRENT);				//Read system ready bit
	 
	while((ready&0x00000001)==0)
	{
		if(waitTime>11)
		{
			return false;
		}
		delay(100);
		waitTime++;
	}
	
	SPI_Write_32(REG_MS_ACAL_CFG, 0x00000013);
	return true;
}

bool ADE9153AClass::StartAcal_AITurbo(void)
{
	uint32_t ready = 0;
	int waitTime = 0;
	 
	while((ready&0x00000001)==0)
	{
		ready = SPI_Read_32(REG_MS_STATUS_CURRENT);		//Read system ready bit
		if(waitTime>15)
		{
			return false;
		}
		delay(100);
		waitTime++;
	} 
	
	SPI_Write_32(REG_MS_ACAL_CFG, 0x00000017);
	return true;
}

bool ADE9153AClass::StartAcal_AV(void)
{
	uint32_t ready;
	int waitTime = 0;
	 
	while((ready&0x00000001)==0)
	{
		ready = SPI_Read_32(REG_MS_STATUS_CURRENT);		//Read system ready bit
		if(waitTime>15)
		{
			return false;
		}
		delay(100);
		waitTime++;
	}
	
	SPI_Write_32(REG_MS_ACAL_CFG, 0x00000043);
	return true;
}

void ADE9153AClass::StopAcal(void)
{
	SPI_Write_32(REG_MS_ACAL_CFG, 0x00000000);
}

bool ADE9153AClass::ApplyAcal(float AICC, float AVCC)
{
	int32_t AIGAIN;
	int32_t AVGAIN;
	
	AIGAIN = (AICC / (CAL_IRMS_CC*1000) - 1) * 134217728;
	AVGAIN = (AVCC / (CAL_VRMS_CC*1000) - 1) * 134217728;
	
	SPI_Write_32(REG_AIGAIN, AIGAIN);
	SPI_Write_32(REG_AVGAIN, AVGAIN);
}

/* 
Description: Starts a new acquisition cycle. Waits for constant time and returns register value and temperature in Degree Celsius
Input:	Structure name
Output: Register reading and temperature value in Degree Celsius
*/

void ADE9153AClass:: ReadTemperature(Temperature *Data)
{
	uint32_t trim;
	uint16_t gain;
	uint16_t offset;
	uint16_t tempReg; 
	float tempValue;
	
	SPI_Write_16(REG_TEMP_CFG,ADE9153A_TEMP_CFG);//Start temperature acquisition cycle
	delay(10); //delay of 2ms. Increase delay if TEMP_TIME is changed

	trim = SPI_Read_32(REG_TEMP_TRIM);
	gain= (trim & 0xFFFF);  //Extract 16 LSB
	offset= ((trim>>16)&0xFFFF); //Extract 16 MSB
	tempReg= SPI_Read_16(REG_TEMP_RSLT);	//Read Temperature result register
	tempValue= ((float)offset / 32.00)-((float)tempReg*(float)gain/(float)131072); 
	
	Data->TemperatureReg=tempReg;
	Data->TemperatureVal=tempValue;
}