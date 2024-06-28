#pragma once




#define SNAUD_MCLK_CTRL 0x00 //Clock/FPLL/low pass filter control
#define SNAUD_FPLL_CTRL 0x01 //FPLL control
#define SNAUD_LDO_CTRL  0x02 //Regulator control


#define SNAUD_FPLL_FRA0 0x04 //FPLL fraction[19:16]
#define SNAUD_FPLL_FRA1 0x05 //FPLL fraction[15:8]
#define SNAUD_FPLL_FRA2 0x06 //FPLL fraction[7:0]
#define SNAUD_FPLL_INT 0x07  //FPLL integer[3:0]
#define SNAUD_I2S_TX_CTRL0 0x08 //I2S transmitter control 0
#define SNAUD_I2S_TX_CTRL1 0x09 //I2S transmitter control 1
#define SNAUD_I2S_RX_CTRL0 0x0A //I2S receiver control 0
#define SNAUD_I2S_RX_CTRL1 0x0B //I2S receiver control 1
#define SNAUD_SWRST_CTRL 0x0C  //Software reset control
#define SNAUD_WP_CTRL 0x0D     //Write protect control


#define SNAUD_I2C_ALTET_ADDR 0x0F //I2C device alternate address
#define SNAUD_ADC_MIC_CTRL 0x10   //ADC microphone bias control
#define SNAUD_ADC_CTRL0 0x11      //ADC control 0
#define SNAUD_ADC_CTRL1 0x12      //ADC control 1
#define SNAUD_ADC_CTRL2 0x13      //ADC control 2
#define SNAUD_ADC_ZCU_CTRL 0x14   //ADC zero-crossing control


#define SNAUD_ADC_GAIN 0x17       //ADC gain control
#define SNAUD_ADC_FILTER 0x18     //ADC digital filter
#define SNAUD_ADC_ADJ 0x19        //ADC adjustment


#define SNAUD_CLASS_D_PWM_CTRL0 0x40  //Class-D amplifier with speaker driver control 0
#define SNAUD_CLASS_D_PWM_CTRL1 0x41  //Class-D amplifier with speaker driver control 1
#define SNAUD_CLASS_D_PWM_CTRL2 0x42 //Class-D amplifier with speaker driver control 2