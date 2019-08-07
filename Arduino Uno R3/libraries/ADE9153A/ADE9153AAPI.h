/**************************************************************************
  ADE9153AAPI.h - Library for ADE9153A energy measurement IC with mSure
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
#ifndef ADE9153AAPI_h
#define ADE9153AAPI_h

#include "Arduino.h"
#include "ADE9153A.h"

/**************************************************************************
  Definitions
 **************************************************************************/
/* Configuration Registers */
#define ADE9153A_AI_PGAGAIN 0x000A		/*Signal on IAN, current channel gain=16x*/
#define ADE9153A_CONFIG0 0x00000000		/*Datapath settings at default*/
#define ADE9153A_CONFIG1 0x0300			/*Chip settings at default*/
#define ADE9153A_CONFIG2 0x0C00			/*High-pass filter corner, fc=0.625Hz*/
#define ADE9153A_CONFIG3 0x0000			/*Peak and overcurrent settings*/
#define ADE9153A_ACCMODE 0x0010			/*Energy accumulation modes, Bit 4, 0 for 50Hz, 1 for 60Hz*/

#define ADE9153A_VLEVEL 0x002C11E8		/*Assuming Vnom=1/2 of fullscale*/
#define ADE9153A_ZX_CFG 0x0000			/*ZX low-pass filter select*/
#define ADE9153A_MASK 0x00000100		/*Enable EGYRDY interrupt*/

#define ADE9153A_ACT_NL_LVL 0x000033C8
#define ADE9153A_REACT_NL_LVL 0x000033C8
#define ADE9153A_APP_NL_LVL 0x000033C8

/* Constant Definitions */
#define ADE9153A_RUN_ON 0x0001			/*DSP On*/
#define ADE9153A_COMPMODE 0x0005		/*Initialize for proper operation*/

#define ADE9153A_VDIV_RSMALL 0x03E8		/*Small resistor on board is 1kOhm=0x3E8*/

/* Energy Accumulation Settings */
#define ADE9153A_EP_CFG 0x0009			/*Energy accumulation configuration*/
#define ADE9153A_EGY_TIME 0x0F9F		/*Accumulate energy for 4000 samples*/

/* Temperature Sensor Settings */
#define ADE9153A_TEMP_CFG 0x000C		/*Temperature sensor configuration*/

/* Ideal Calibration Values for ADE9153A Shield Based on Sensor Values */
#define CAL_IRMS_CC		0.838190	// (uA/code)
#define CAL_VRMS_CC		13.41105	// (uV/code)	
#define CAL_POWER_CC 	1508.743	// (uW/code) Applicable for Active, reactive and apparent power
#define CAL_ENERGY_CC	0.858307	// (uWhr/xTHR_HI code)Applicable for Active, reactive and apparent energy

/**************************************************************************
  Structures and Global Variables
 **************************************************************************/

struct EnergyRegs {
	int32_t ActiveEnergyReg;
	int32_t FundReactiveEnergyReg;
	int32_t ApparentEnergyReg;
	float ActiveEnergyValue;
	float FundReactiveEnergyValue;
	float ApparentEnergyValue;
};

struct PowerRegs {
	int32_t ActivePowerReg;
	float ActivePowerValue;
	int32_t FundReactivePowerReg;
	float FundReactivePowerValue;
	int32_t ApparentPowerReg;
	float ApparentPowerValue;
};

struct RMSRegs {
	int32_t CurrentRMSReg;
	float CurrentRMSValue;
	int32_t VoltageRMSReg;
	float VoltageRMSValue;
};

struct HalfRMSRegs {
	int32_t HalfCurrentRMSReg;
	float HalfCurrentRMSValue;
	int32_t HalfVoltageRMSReg;
	float HalfVoltageRMSValue;
};

struct PQRegs {
	int32_t PowerFactorReg;
	float PowerFactorValue;
	int32_t PeriodReg;
	float FrequencyValue;
	int32_t AngleReg_AV_AI;
	float AngleValue_AV_AI;
};

struct AcalRegs {
	int32_t AcalAICCReg;
	float AICC;
	int32_t AcalAICERTReg;
	int32_t AcalAVCCReg;
	float AVCC;
	int32_t AcalAVCERTReg;
};

struct Temperature {
	uint16_t TemperatureReg;
	float TemperatureVal;
};

class ADE9153AClass
{
public:
	ADE9153AClass(void);
	void SetupADE9153A(void);

	/* SPI Functions */
	bool SPI_Init(uint32_t SPI_speed, uint8_t chipSelect_Pin);
	void SPI_Write_16(uint16_t Address, uint16_t Data );
	void SPI_Write_32(uint16_t Address, uint32_t Data );
	uint16_t SPI_Read_16(uint16_t Address);
	uint32_t SPI_Read_32(uint16_t Address);

	/* ADE9153A Calculated Paramter Read Functions */
	void ReadEnergyRegs(EnergyRegs *Data);
	void ReadPowerRegs(PowerRegs *Data);
	void ReadRMSRegs(RMSRegs *Data);
	void ReadHalfRMSRegs(HalfRMSRegs *Data);
	void ReadPQRegs(PQRegs *Data);

	void ReadAcalRegs(AcalRegs *Data);
	bool StartAcal_AINormal(void);
	bool StartAcal_AITurbo(void);
	bool StartAcal_AV(void);
	void StopAcal(void);
	bool ApplyAcal(float AICC, float AVCC);

	void ReadTemperature(Temperature *Data);

private:
	uint8_t _chipSelect_Pin;
};

#endif