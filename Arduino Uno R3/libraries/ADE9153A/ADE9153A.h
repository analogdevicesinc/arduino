/* ================================================================================
     Project      :   ADE9153A
     File         :   ADE9153A.h
     Description  :   C header file contains macros for Registers' address relative to instances and plain bit-fields.

     Date         :   Jan 25, 2018

     Copyright (c) 2018 Analog Devices, Inc.  All Rights Reserved.
     This software is proprietary and confidential to Analog Devices, Inc. and
     its licensors.

     This file was auto-generated. Do not make local changes to this file.
  
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
   ================================================================================ */

#ifndef ADE9153A_H
#define ADE9153A_H

#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */


#define MASK_ADE9153A                                                       0xFFFF
#define REG_AIGAIN                                                          0x0000    /* Phase A current gain adjust. */
#define REG_APHASECAL                                                       0x0001    /* Phase A phase correction factor. */
#define REG_AVGAIN                                                          0x0002    /* Phase A voltage gain adjust. */
#define REG_AIRMS_OS                                                        0x0003    /* Phase A current rms offset for filter-based AIRMS calculation. */
#define REG_AVRMS_OS                                                        0x0004    /* Phase A voltage rms offset for filter-based AVRMS calculation. */
#define REG_APGAIN                                                          0x0005    /* Phase A power gain adjust for AWATT, AVA, and AFVAR calculations. */
#define REG_AWATT_OS                                                        0x0006    /* Phase A total active power offset correction for AWATT calculation. */
#define REG_AFVAR_OS                                                        0x0007    /* Phase A fundamental reactive power offset correction for AFVAR calculation. */
#define REG_AVRMS_OC_OS                                                     0x0008    /* Phase A voltage rms offset for fast rms, AVRMS_OC calculation. */
#define REG_AIRMS_OC_OS                                                     0x0009    /* Phase A current rms offset for fast rms, AIRMS_OC calculation. */
#define REG_BIGAIN                                                          0x0010    /* Phase B current gain adjust. */
#define REG_BIRMS_OS                                                        0x0013    /* Phase B current rms offset for filter-based BIRMS calculation. */
#define REG_BIRMS_OC_OS                                                     0x0019    /* Phase B current rms offset for fast rms, BIRMS_OC calculation. */
#define REG_CONFIG0                                                         0x0020    /* DSP configuration register. */
#define REG_VNOM                                                            0x0021    /* Nominal phase voltage rms used in the calculation of apparent power, AVA, when the VNOMA_EN bit is set in the CONFIG0 register. */
#define REG_DICOEFF                                                         0x0022    /* Value used in the digital integrator algorithm. If the integrator is turned on, with INTEN_BI equal to 1 in the CONFIG0 register, it is recommended to leave this register at the default value. */
#define REG_BI_PGAGAIN                                                      0x0023    /* PGA gain for Current Channel B ADC. */
#define REG_MS_ACAL_CFG                                                     0x0030    /* MSure autocalibration configuration register. */
#define REG_CT_PHASE_DELAY                                                  0x0049    /* Phase delay of the CT used on Current Channel B. This register is in 5.27 format and expressed in degrees. */
#define REG_CT_CORNER                                                       0x004A    /* Corner frequency of the CT. This value is calculated from the CT_PHASE_DELAY value. */
#define REG_VDIV_RSMALL                                                     0x004C    /* This register holds the resistance value, in Ω, of the small resistor in the resistor divider. */
#define REG_AI_WAV                                                          0x0200    /* Instantaneous Current Channel A waveform processed by the DSP, at 4kSPS. */
#define REG_AV_WAV                                                          0x0201    /* Instantaneous Voltage Channel waveform processed by the DSP, at 4kSPS. */
#define REG_AIRMS                                                           0x0202    /* Phase A filter-based current rms value updated at 4kSPS. */
#define REG_AVRMS                                                           0x0203    /* Phase A filter-based voltage rms value updated at 4kSPS. */
#define REG_AWATT                                                           0x0204    /* Phase A low-pass filtered total active power updated at 4kSPS. */
#define REG_AVA                                                             0x0206    /* Phase A total apparent power updated at 4kSPS. */
#define REG_AFVAR                                                           0x0207    /* Phase A fundamental reactive power updated at 4kSPS. */
#define REG_APF                                                             0x0208    /* Phase A power factor updated at 1.024 sec. */
#define REG_AIRMS_OC                                                        0x0209    /* Phase A current fast rms calculation; one cycle rms updated every half cycle. */
#define REG_AVRMS_OC                                                        0x020A    /* Phase A voltage fast rms calculation; one cycle rms updated every half cycle. */
#define REG_BI_WAV                                                          0x0210    /* Instantaneous Phase B Current Channel waveform processed by the DSP at 4kSPS. */
#define REG_BIRMS                                                           0x0212    /* Phase B filter-based current rms value updated at 4kSPS. */
#define REG_BIRMS_OC                                                        0x0219    /* Phase B Current fast rms calculation; one cycle rms updated every half cycle. */
#define REG_MS_ACAL_AICC                                                    0x0220    /* Current Channel A mSure CC estimation from autocalibration. */
#define REG_MS_ACAL_AICERT                                                  0x0221    /* Current Channel A mSure certainty of autocalibration. */
#define REG_MS_ACAL_BICC                                                    0x0222    /* Current Channel B mSure CC estimation from autocalibration. */
#define REG_MS_ACAL_BICERT                                                  0x0223    /* Current Channel B mSure certainty of autocalibration. */
#define REG_MS_ACAL_AVCC                                                    0x0224    /* Voltage Channel mSure CC estimation from autocalibration. */
#define REG_MS_ACAL_AVCERT                                                  0x0225    /* Voltage Channel mSure certainty of autocalibration. */
#define REG_MS_STATUS_CURRENT                                               0x0240    /* The MS_STATUS_CURRENT register contains bits that reflect the present state of the mSure system. */
#define REG_VERSION_DSP                                                     0x0241    /* This register indicates the version of the ADE9153A DSP after the user writes RUN=1 to start measurements. */
#define REG_VERSION_PRODUCT                                                 0x0242    /* This register indicates the version of the product being used. */
#define REG_AWATT_ACC                                                       0x039D    /* Phase A accumulated total active power; updated after PWR_TIME 4kSPS samples. */
#define REG_AWATTHR_LO                                                      0x039E    /* Phase A accumulated total active energy, least significant bits (LSBs). Updated according to the settings in the EP_CFG and EGY_TIME registers. */
#define REG_AWATTHR_HI                                                      0x039F    /* Phase A accumulated total active energy, most significant bits (MSBs). Updated according to the settings in the EP_CFG and EGY_TIME registers. */
#define REG_AVA_ACC                                                         0x03B1    /* Phase A accumulated total apparent power; updated after PWR_TIME 4kSPS samples. */
#define REG_AVAHR_LO                                                        0x03B2    /* Phase A accumulated total apparent energy, LSBs. Updated according to the settings in the EP_CFG and EGY_TIME registers. */
#define REG_AVAHR_HI                                                        0x03B3    /* Phase A accumulated total apparent energy, MSBs. Updated according to the settings in the EP_CFG and EGY_TIME registers. */
#define REG_AFVAR_ACC                                                       0x03BB    /* Phase A accumulated fundamental reactive power; updated after PWR_TIME 4kSPS samples. */
#define REG_AFVARHR_LO                                                      0x03BC    /* Phase A accumulated fundamental reactive energy, LSBs. Updated according to the settings in the EP_CFG and EGY_TIME registers. */
#define REG_AFVARHR_HI                                                      0x03BD    /* Phase A accumulated fundamental reactive energy, MSBs. Updated according to the settings in the EP_CFG and EGY_TIME registers. */
#define REG_PWATT_ACC                                                       0x03EB    /* Accumulated positive total active power from the AWATT register; updated after PWR_TIME 4 kSPS samples. */
#define REG_NWATT_ACC                                                       0x03EF    /* Accumulated negative total active power from the AWATT register; updated after PWR_TIME 4 kSPS samples. */
#define REG_PFVAR_ACC                                                       0x03F3    /* Accumulated positive fundamental reactive power from the AFVAR register; updated after PWR_TIME 4 kSPS samples. */
#define REG_NFVAR_ACC                                                       0x03F7    /* Accumulated negative fundamental reactive power from the AFVAR register, updated after PWR_TIME 4 kSPS samples. */
#define REG_IPEAK                                                           0x0400    /* Current peak register. */
#define REG_VPEAK                                                           0x0401    /* Voltage peak register. */
#define REG_STATUS                                                          0x0402    /* Tier 1 interrupt status register. */
#define REG_MASK                                                            0x0405    /* Tier 1 interrupt enable register. */
#define REG_OI_LVL                                                          0x0409    /* Overcurrent RMS_OC detection threshold level. */
#define REG_OIA                                                             0x040A    /* Phase A overcurrent RMS_OC value. If overcurrent detection on this channel is enabled with OIA_EN in the CONFIG3 register and AIRMS_OC is greater than the OILVL threshold, this value is updated. */
#define REG_OIB                                                             0x040B    /* Phase B overcurrent RMS_OC value. See the OIA description. */
#define REG_USER_PERIOD                                                     0x040E    /* User configured line period value used for RMS_OC when the UPERIOD_SEL bit in the CONFIG2 register is set. */
#define REG_VLEVEL                                                          0x040F    /* Register used in the algorithm that computes the fundamental reactive power. */
#define REG_DIP_LVL                                                         0x0410    /* Voltage RMS_OC dip detection threshold level. */
#define REG_DIPA                                                            0x0411    /* Phase A voltage RMS_OC value during a dip condition. */
#define REG_SWELL_LVL                                                       0x0414    /* Voltage RMS_OC swell detection threshold level. */
#define REG_SWELLA                                                          0x0415    /* Phase A voltage RMS_OC value during a swell condition. */
#define REG_APERIOD                                                         0x0418    /* Line period on the Phase A voltage. */
#define REG_ACT_NL_LVL                                                      0x041C    /* No load threshold in the total active power datapath. */
#define REG_REACT_NL_LVL                                                    0x041D    /* No load threshold in the fundamental reactive power datapath. */
#define REG_APP_NL_LVL                                                      0x041E    /* No load threshold in the total apparent power datapath. */
#define REG_PHNOLOAD                                                        0x041F    /* Phase no load register. */
#define REG_WTHR                                                            0x0420    /* Sets the maximum output rate from the digital to frequency converter of the total active power for the CF calibration pulse output. It is recommended to leave this at WTHR = 0x00100000. */
#define REG_VARTHR                                                          0x0421    /* See WTHR. It is recommended to leave this value at VARTHR = 0x00100000. */
#define REG_VATHR                                                           0x0422    /* See WTHR. It is recommended to leave this value at VATHR = 0x00100000. */
#define REG_LAST_DATA_32                                                    0x0423    /* This register holds the data read or written during the last 32-bit transaction on the SPI port. */
#define REG_CF_LCFG                                                         0x0425    /* CF calibration pulse width configuration register. */
#define REG_TEMP_TRIM                                                       0x0471    /* Temperature sensor gain and offset, calculated during the manufacturing process. */
#define REG_CHIP_ID_HI                                                      0x0472    /* Chip identification, 32 MSBs. */
#define REG_CHIP_ID_LO                                                      0x0473    /* Chip identification, 32 LSBs. */
/* 16-bit below */
#define REG_RUN                                                             0x0480    /* Write this register to 1 to start the measurements. */
#define REG_CONFIG1                                                         0x0481    /* Configuration Register 1. */
#define REG_ANGL_AV_AI                                                      0x0485    /* Time between positive to negative zero crossings on Phase A voltage and current. */
#define REG_ANGL_AI_BI                                                      0x0488    /* Time between positive to negative zero crossings on Phase A and Phase B currents. */
#define REG_DIP_CYC                                                         0x048B    /* Voltage RMS_OC dip detection cycle configuration. */
#define REG_SWELL_CYC                                                       0x048C    /* Voltage RMS_OC swell detection cycle configuration. */
#define REG_CFMODE                                                          0x0490    /* CFx configuration register. */
#define REG_COMPMODE                                                        0x0491    /* Computation mode register. Set this register to 0x0005. */
#define REG_ACCMODE                                                         0x0492    /* Accumulation mode register. */
#define REG_CONFIG3                                                         0x0493    /* Configuration Register 3 for configuration of power quality settings. */
#define REG_CF1DEN                                                          0x0494    /* CF1 denominator register. */
#define REG_CF2DEN                                                          0x0495    /* CF2 denominator register. */
#define REG_ZXTOUT                                                          0x0498    /* Zero-crossing timeout configuration register. */
#define REG_ZXTHRSH                                                         0x0499    /* Voltage channel zero-crossing threshold register. */
#define REG_ZX_CFG                                                          0x049A    /* Zero-crossing detection configuration register. */
#define REG_PHSIGN                                                          0x049D    /* Power sign register. */
#define REG_CRC_RSLT                                                        0x04A8    /* This register holds the CRC of the configuration registers. */
#define REG_CRC_SPI                                                         0x04A9    /* The register holds the 16-bit CRC of the data sent out on the MOSI/RX pin during the last SPI register read. */
#define REG_LAST_DATA_16                                                    0x04AC    /* This register holds the data read or written during the last 16-bit transaction on the SPI port. When using UART, this register holds the lower 16 bits of the last data read or write. */
#define REG_LAST_CMD                                                        0x04AE    /* This register holds the address and the read/write operation request (CMD_HDR) for the last transaction on the SPI port. */
#define REG_CONFIG2                                                         0x04AF    /* Configuration Register 2. This register controls the high-pass filter (HPF) corner and the user period selection. */
#define REG_EP_CFG                                                          0x04B0    /* Energy and power accumulation configuration. */
#define REG_PWR_TIME                                                        0x04B1    /* Power update time configuration. */
#define REG_EGY_TIME                                                        0x04B2    /* Energy accumulation update time configuration. */
#define REG_CRC_FORCE                                                       0x04B4    /* This register forces an update of the CRC of configuration registers. */
#define REG_TEMP_CFG                                                        0x04B6    /* Temperature sensor configuration register. */
#define REG_TEMP_RSLT                                                       0x04B7    /* Temperature measurement result. */
#define REG_AI_PGAGAIN                                                      0x04B9    /* This register configures the PGA gain for Current Channel A. */
#define REG_WR_LOCK                                                         0x04BF    /* This register enables the configuration lock feature. */
#define REG_MS_STATUS_IRQ                                                   0x04C0    /* Tier 2 status register for the autocalibration and monitoring mSure system related interrupts. Any bit set in this register causes the corresponding bit in the status register to be set. This register is cleared on a read and all bits are reset. If a new status bit arrives on the same clock on which the read occurs, the new status bit remains set; in this way, no status bit is missed. */
#define REG_EVENT_STATUS                                                    0x04C1    /* Tier 2 status register for power quality event related interrupts. See the MS_STATUS_IRQ description. */
#define REG_CHIP_STATUS                                                     0x04C2    /* Tier 2 status register for chip error related interrupts. See the MS_STATUS_IRQ description. */
#define REG_UART_BAUD_SWITCH                                                0x04DC    /* This register switches the UART Baud rate between 4800 and 115,200 Baud. Writing a value of 0x0052 sets the Baud rate to 115,200 Baud; any other value maintains a Baud rate of 4800. */
#define REG_VERSION                                                         0x04FE    /* Version of the ADE9153 IC. */
#define REG_AI_WAV_1                                                        0x0600    /* SPI burst read accessible registers organized functionally. See AI_WAV. */
#define REG_AV_WAV_1                                                        0x0601    /* SPI burst read accessible registers organized functionally. See AV_WAV. */
#define REG_BI_WAV_1                                                        0x0602    /* SPI burst read accessible registers organized functionally. See BI_WAV. */
#define REG_AIRMS_1                                                         0x0604    /* SPI burst read accessible registers organized functionally. See AIRMS. */
#define REG_BIRMS_1                                                         0x0605    /* SPI burst read accessible registers organized functionally. See BIRMS. */
#define REG_AVRMS_1                                                         0x0606    /* SPI burst read accessible registers organized functionally. See AVRMS. */
#define REG_AWATT_1                                                         0x0608    /* SPI burst read accessible registers organized functionally. See AWATT. */
#define REG_AFVAR_1                                                         0x060A    /* SPI burst read accessible registers organized functionally. See AFVAR. */
#define REG_AVA_1                                                           0x060C    /* SPI burst read accessible registers organized functionally. See AVA. */
#define REG_APF_1                                                           0x060E    /* SPI burst read accessible registers organized functionally. See APF. */
#define REG_AI_WAV_2                                                        0x0610    /* SPI burst read accessible registers organized by phase. See AI_WAV. */
#define REG_AV_WAV_2                                                        0x0611    /* SPI burst read accessible registers organized by phase. See AV_WAV. */
#define REG_AIRMS_2                                                         0x0612    /* SPI burst read accessible registers organized by phase. See AIRMS. */
#define REG_AVRMS_2                                                         0x0613    /* SPI burst read accessible registers organized by phase. See AVRMS. */
#define REG_AWATT_2                                                         0x0614    /* SPI burst read accessible registers organized by phase. See AWATT. */
#define REG_AVA_2                                                           0x0615    /* SPI burst read accessible registers organized by phase. See AVA. */
#define REG_AFVAR_2                                                         0x0616    /* SPI burst read accessible registers organized by phase. See AFVAR. */
#define REG_APF_2                                                           0x0617    /* SPI burst read accessible registers organized by phase. See APF. */
#define REG_BI_WAV_2                                                        0x0618    /* SPI burst read accessible registers organized by phase. See BI_WAV. */
#define REG_BIRMS_2                                                         0x061A    /* SPI burst read accessible registers organized by phase. See BIRMS. */

#endif  /* end ifndef ADE9153A_H */
