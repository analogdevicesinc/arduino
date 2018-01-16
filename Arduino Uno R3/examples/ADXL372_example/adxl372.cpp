/***************************************************************************//**
 *   @file   adxl372.c
 *   @brief  ADXL372 device driver.
********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*******************************************************************************/
#include "adxl372.h"
#include "Communication.h"

//#define ADXL_DEBUG

static int adxl_read_reg_multiple(adxl_spi_handle *spi, unsigned char reg,
                                  unsigned char count, unsigned char *val)
{
    reg = reg << 1 | ADXL_SPI_RNW;
    return spi_write_then_read(spi, &reg, 1, val, count);   
}

int adxl_read_reg(adxl_spi_handle *spi,
                         unsigned char reg, unsigned char *val)
{
    return adxl_read_reg_multiple(spi, reg, 1, val);
}

int adxl_write_reg(adxl_spi_handle *spi, unsigned char reg,
                          unsigned char val)
{
    unsigned char tmp[2];
    tmp[0] = reg << 1;
    tmp[1] = val;

#ifdef ADXL_DEBUG
    spi_write_then_read(spi, tmp, 2, NULL, 0);
        adxl_delay(1);
    adxl_read_reg(spi, reg, tmp);
    if (val != tmp[0])
      dnm_ucli_printf("verify failed REG 0x%X - 0x%X != 0x%X\r\n", reg, val, tmp[0]);
    
    return 0;
    
#else
    return spi_write_then_read(spi, tmp, 2, NULL, 0); 
#endif    
}

static int adxl_update_reg(adxl_spi_handle *spi, unsigned char reg,
                           unsigned char mask, unsigned char shift, unsigned char val)
{
    unsigned char tmp;
    int err;

    err = adxl_read_reg(spi, reg, &tmp);
    if (err < 0)
        return err;

    tmp &= mask;
    tmp |= (val << shift) & ~mask;

    return adxl_write_reg(spi, reg, tmp);
}

int adxl372_Set_Op_mode(struct adxl372_device *dev, ADXL372_OP_MODE mode)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_POWER_CTL, PWRCTRL_OPMODE_MASK, 0,
                           mode);
}

int adxl372_Set_ODR(struct adxl372_device *dev, ADXL372_ODR odr)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_TIMING, TIMING_ODR_MASK,
                           TIMING_ODR_POS, odr);
}

int adxl372_Set_WakeUp_Rate(struct adxl372_device *dev, ADXL372_WUR wur)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_TIMING, TIMING_WUR_MASK,
                           TIMING_WUR_POS, wur);
}

int adxl372_Set_BandWidth(struct adxl372_device *dev, ADXL372_BW bw)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_MEASURE, MEASURE_BANDWIDTH_MASK, 0,
                           bw);
}

int adxl372_Set_Autosleep(struct adxl372_device *dev, bool enable)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_MEASURE, MEASURE_AUTOSLEEP_MASK,
                           MEASURE_AUTOSLEEP_POS, enable);
}

int adxl372_Set_Act_Proc_Mode(struct adxl372_device *dev,
                              ADXL372_ACT_PROC_MODE mode)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_MEASURE, MEASURE_ACTPROC_MASK,
                           MEASURE_ACTPROC_POS, mode);
}

int adxl372_Set_InstaOn_Thresh(struct adxl372_device *dev,
                               ADXL_INSTAON_THRESH mode)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_POWER_CTL,
                           PWRCTRL_INSTON_THRESH_MASK, INSTAON_THRESH_POS, mode);
}

int adxl372_Set_Activity_Threshold(struct adxl372_device *dev,
                                   unsigned short thresh, bool referenced, bool enable)
{
    int err = adxl372_Set_Op_mode(dev, STAND_BY); /* FIXME ? */
    if (err < 0)
        return err;

    adxl_write_reg(dev->spi, ADI_ADXL372_X_THRESH_ACT_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_X_THRESH_ACT_L,
                   (thresh << 5) | (referenced << 1) | enable);
    adxl_write_reg(dev->spi, ADI_ADXL372_Y_THRESH_ACT_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_Y_THRESH_ACT_L, (thresh << 5) | enable);
    adxl_write_reg(dev->spi, ADI_ADXL372_Z_THRESH_ACT_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_Z_THRESH_ACT_L, (thresh << 5) | enable);

    return err;
}

int adxl372_Set_Activity2_Threshold(struct adxl372_device *dev,
                                   unsigned short thresh, bool referenced, bool enable)
{
    int err = adxl372_Set_Op_mode(dev, STAND_BY); /* FIXME ? */
    if (err < 0)
        return err;

    adxl_write_reg(dev->spi, ADI_ADXL372_X_THRESH_ACT2_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_X_THRESH_ACT2_L,
                   (thresh << 5) | (referenced << 1) | enable);
    adxl_write_reg(dev->spi, ADI_ADXL372_Y_THRESH_ACT2_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_Y_THRESH_ACT2_L, (thresh << 5) | enable);
    adxl_write_reg(dev->spi, ADI_ADXL372_Z_THRESH_ACT2_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_Z_THRESH_ACT2_L, (thresh << 5) | enable);

    return err;
}

int adxl372_Set_Inactivity_Threshold(struct adxl372_device *dev,
                                     unsigned short thresh, bool referenced, bool enable)
{
    int err = adxl372_Set_Op_mode(dev, STAND_BY);  /* FIXME ? */
    if (err < 0)
        return err;

    adxl_write_reg(dev->spi, ADI_ADXL372_X_THRESH_INACT_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_X_THRESH_INACT_L,
                   (thresh << 5) | (referenced << 1) | enable);
    adxl_write_reg(dev->spi, ADI_ADXL372_Y_THRESH_INACT_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_Y_THRESH_INACT_L, (thresh << 5) | enable);
    adxl_write_reg(dev->spi, ADI_ADXL372_Z_THRESH_INACT_H, thresh >> 3);
    adxl_write_reg(dev->spi, ADI_ADXL372_Z_THRESH_INACT_L, (thresh << 5) | enable);

    return err;
}

int adxl372_Set_Activity_Time(struct adxl372_device *dev, unsigned char time)
{
    return adxl_write_reg(dev->spi, ADI_ADXL372_TIME_ACT, time);
}

int adxl372_Set_Inactivity_Time(struct adxl372_device *dev, unsigned short time)
{
    adxl_write_reg(dev->spi, ADI_ADXL372_TIME_INACT_H, time >> 8);

    return adxl_write_reg(dev->spi, ADI_ADXL372_TIME_INACT_L, time & 0xFF);
}

int adxl372_Set_Filter_Settle(struct adxl372_device *dev,
                              ADXL372_Filter_Settle mode)
{
    return adxl_update_reg(dev->spi, ADI_ADXL372_POWER_CTL,
                           PWRCTRL_FILTER_SETTLE_MASK, PWRCTRL_FILTER_SETTLE_POS, mode);
}

int adxl372_Get_DevID(struct adxl372_device *dev, unsigned char *DevID)
{
    return adxl_read_reg_multiple(dev->spi, ADI_ADXL372_ADI_DEVID, 1, DevID);
}

int adxl372_Get_Status_Register(struct adxl372_device *dev,
                                unsigned char *adxl_status)
{
    return adxl_read_reg(dev->spi, ADI_ADXL372_STATUS_1, adxl_status);
}

int adxl372_Get_ActivityStatus_Register(struct adxl372_device *dev,
                                        unsigned char *adxl_status)
{
    return adxl_read_reg(dev->spi, ADI_ADXL372_STATUS_2, adxl_status);
}

#define SWAP16(x) ((x) = (((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8))
#define SHIFT4(x) ((x) = (x) >> 4)

int adxl372_Get_Highest_Peak_Accel_data(struct adxl372_device *dev,
                                        AccelTriplet_t *max_peak)
{
    int err = adxl_read_reg_multiple(dev->spi, ADI_ADXL372_X_MAXPEAK_H, 6,
                                     (unsigned char *) max_peak);

#ifdef L_ENDIAN
    SWAP16(max_peak->x);
    SWAP16(max_peak->y);
    SWAP16(max_peak->z);
#endif

    SHIFT4(max_peak->x);
    SHIFT4(max_peak->y);
    SHIFT4(max_peak->z);

    return err;
}

int adxl372_Get_Accel_data(struct adxl372_device *dev,
                           AccelTriplet_t *accel_data)
{
    unsigned char status;
    int err;
    
    do {
        adxl372_Get_Status_Register(dev, &status);
    } while (!(status & DATA_RDY));

    err = adxl_read_reg_multiple(dev->spi, ADI_ADXL372_X_DATA_H, 6,
                                 (unsigned char *) accel_data);

#ifdef L_ENDIAN
  
    SWAP16(accel_data->x);
    SWAP16(accel_data->y);
    SWAP16(accel_data->z);
#endif

    SHIFT4(accel_data->x);
    SHIFT4(accel_data->y);
    SHIFT4(accel_data->z);    

    return err;
}

int adxl372_Reset(struct adxl372_device *dev)
{
    int err = adxl_write_reg(dev->spi, ADI_ADXL372_SRESET, 0x52);

    delay(1);

    return err;
}

int adxl372_Configure_FIFO(struct adxl372_device *dev,
                           unsigned short fifo_samples,
                           ADXL372_FIFO_MODE fifo_mode,
                           ADXL372_FIFO_FORMAT fifo_format)
{
    unsigned char config;
    int err;

    adxl372_Set_Op_mode(dev, STAND_BY);

    if (fifo_samples > 512)
        return -1;

    fifo_samples -= 1;

    config = ((unsigned char)fifo_mode << FIFO_CRL_MODE_POS) |
             ((unsigned char)fifo_format << FIFO_CRL_FORMAT_POS) |
             ((fifo_samples > 0xFF) << FIFO_CRL_SAMP8_POS);

    err = adxl_write_reg(dev->spi, ADI_ADXL372_FIFO_SAMPLES, fifo_samples & 0xFF);
    if (err < 0)
        return err;

    err = adxl_write_reg(dev->spi, ADI_ADXL372_FIFO_CTL, config);
    if (err < 0)
        return err;

    dev->fifo_config.samples = fifo_samples + 1;
    dev->fifo_config.mode = fifo_mode;
    dev->fifo_config.format = fifo_format;

    return err;
}

int adxl372_Get_FIFO_data(struct adxl372_device *dev, short *samples)
{
    int err, i;
    if(dev->fifo_config.mode == BYPASSED)
        return -1;

    err = adxl_read_reg_multiple(dev->spi, ADI_ADXL372_FIFO_DATA,
                                 dev->fifo_config.samples * 2, (unsigned char *) samples);

#ifdef L_ENDIAN
    for (i = 0; i < dev->fifo_config.samples; i++)
        SWAP16(samples[i]);
#endif

    return err;
}
