/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "snaud0.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "snaud0_reg.h"

typedef struct {
    i2c_port_t port;
    uint16_t dev_addr;
} snaud0_dev_t;

/*
 * Clock coefficient structure
 */
struct _coeff_div {
    uint32_t mclk;
    uint32_t rate;
    uint8_t fpll_int;
    uint8_t fpll_fra;
    uint8_t fpll_div;
    uint8_t imclkdiv; 
    uint8_t bclk_div;  
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div_snaud02[] = {
    /*!< mclk     rate   FPLL_INT  FPLL_FRA   FPLL_DIV  iMCLKDIV  BCLKDIV  */
    
    /* 8k */
    {8000000, 8000, 0xD, 0x2F1B, 9, 1, 4},
    {10000000, 8000, 0xB, 0x0F27C, 9, 1, 4},
    {12000000, 8000, 0x9, 0x374BC, 9, 1, 4},
    {14000000, 8000, 0x7, 0xE640F, 9, 1, 4},
    {16000000, 8000, 0x6, 0xE978D, 9, 1, 4},

    /* 11.025k */
    {8000000, 11025, 0xD, 0x0F27C, 9, 1, 4},
    {10000000, 11025, 0xB, 0x6872B, 9, 1, 4},
    {12000000, 11025, 0x9, 0x10625, 9, 1, 4},
    {14000000, 11025, 0x8, 0x0E560, 9, 1, 4},

    /* 12k */
    {8000000, 12000, 0xD, 0x374BC, 9, 1, 4},
    {10000000, 12000, 0xB, 0xE640F, 9, 1, 4},
    {12000000, 12000, 0x9, 0xE978D, 9, 1, 4},
    {14000000, 12000, 0x8, 0xB374C, 9, 1, 4},

    /* 16k */
    {8000000, 16000, 0xD, 0x2F1B, 9, 1, 4},
    {10000000, 16000, 0xB, 0x0F27C, 9, 1, 4},
    {12000000, 16000, 0x9, 0x374BC, 9, 1, 4},
    {14000000, 16000, 0x7, 0xE640F, 9, 1, 4},
    {16000000, 16000, 0x6, 0xE978D, 9, 1, 4},

    /* 22.05k */
    {8000000, 22050, 0xD, 0x0F27C, 9, 1, 4},
    {10000000, 22050, 0xB, 0x6872B, 9, 1, 4},
    {12000000, 22050, 0x9, 0x10625, 9, 1, 4},
    {14000000, 22050, 0x8, 0x0E560, 9, 1, 4},

    /* 24k */
    {8000000, 24000, 0xD, 0x374BC, 9, 1, 4},
    {10000000, 24000, 0xB, 0xE640F, 9, 1, 4},
    {12000000, 24000, 0x9, 0xE978D, 9, 1, 4},
    {14000000, 24000, 0x8, 0xB374C, 9, 1, 4},

    /* 32k */
    {8000000, 32000, 0xD, 0x2F1B, 9, 1, 4},
    {10000000, 32000, 0xB, 0x0F27C, 9, 1, 4},
    {12000000, 32000, 0x9, 0x374BC, 9, 1, 4},
    {14000000, 32000, 0x7, 0xE640F, 9, 1, 4},
    {16000000, 32000, 0x6, 0xE978D, 9, 1, 4},

    /* 44.1k */
    {8000000, 44100, 0xD, 0x0F27C, 9, 1, 4},
    {10000000, 44100, 0xB, 0x6872B, 9, 1, 4},
    {12000000, 44100, 0x9, 0x10625, 9, 1, 4},
    {14000000, 44100, 0x8, 0x0E560, 9, 1, 4},

    /* 48k */
    {8000000, 48000, 0xD, 0x2F1B, 9, 1, 4},
    {10000000, 48000, 0xB, 0x0F27C, 9, 1, 4},
    {12000000, 48000, 0x9, 0x374BC, 9, 1, 4},
    {14000000, 48000, 0x7, 0xE640F, 9, 1, 4},
    {16000000, 48000, 0x6, 0xE978D, 9, 1, 4},
};


static const char *TAG = "SNAUD0";

static inline esp_err_t snaud0_write_reg(snaud0_handle_t dev, uint8_t reg_addr, uint8_t data)
{
    snaud0_dev_t *es = (snaud0_dev_t *) dev;
    const uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(es->port, es->dev_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

static inline esp_err_t snaud0_read_reg(snaud0_handle_t dev, uint8_t reg_addr, uint8_t *reg_value)
{
    snaud0_dev_t *es = (snaud0_dev_t *) dev;
    return i2c_master_write_read_device(es->port, es->dev_addr, &reg_addr, 1, reg_value, 1, pdMS_TO_TICKS(1000));
}

/*
* look for the coefficient in coeff_div[] table
*/
static int get_coeff(uint32_t mclk, uint32_t rate)
{
    for (int i = 0; i < (sizeof(coeff_div) / sizeof(coeff_div[0])); i++) {
        if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk) {
            return i;
        }
    }

    return -1;
}

esp_err_t snaud0_sample_frequency_config(snaud0_handle_t dev, int mclk_frequency, int sample_frequency)
{
    uint8_t regv;

    /* Get clock coefficients from coefficient table */
    int coeff = get_coeff(mclk_frequency, sample_frequency);

    if (coeff < 0) {
        ESP_LOGE(TAG, "Unable to configure sample rate %dHz with %dHz MCLK", sample_frequency, mclk_frequency);
        return ESP_ERR_INVALID_ARG;
    }

    const struct _coeff_div *const selected_coeff = &coeff_div[coeff];

    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG02, regv), TAG, "I2C read/write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG03, reg03), TAG, "I2C read/write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG04, selected_coeff->dac_osr), TAG, "I2C read/write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG05, reg05), TAG, "I2C read/write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG06, regv), TAG, "I2C read/write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG07, regv), TAG, "I2C read/write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG08, selected_coeff->lrck_l), TAG, "I2C read/write error");

    return ESP_OK;
}

static esp_err_t snaud0_clock_config(snaud0_handle_t dev, const snaud0_clock_config_t *const clk_cfg, snaud0_resolution_t res)
{
    uint8_t reg06;
    uint8_t reg01 = 0x3F; // Enable all clocks
    int mclk_hz;

    /* Select clock source for internal MCLK and determine its frequency */
    if (clk_cfg->mclk_from_mclk_pin) {
        mclk_hz = clk_cfg->mclk_frequency;
    } else {
        mclk_hz = clk_cfg->sample_frequency * (int)res * 2;
        reg01 |= BIT(7); // Select BCLK (a.k.a. SCK) pin
    }

    if (clk_cfg->mclk_inverted) {
        reg01 |= BIT(6); // Invert MCLK pin
    }
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG01, reg01), TAG, "I2C read/write error");

    ESP_RETURN_ON_ERROR(snaud0_read_reg(dev, ES8311_CLK_MANAGER_REG06, &reg06), TAG, "I2C read/write error");
    if (clk_cfg->sclk_inverted) {
        reg06 |= BIT(5);
    } else {
        reg06 &= ~BIT(5);
    }
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_CLK_MANAGER_REG06, reg06), TAG, "I2C read/write error");

    /* Configure clock dividers */
    return snaud0_sample_frequency_config(dev, mclk_hz, clk_cfg->sample_frequency);
}

static esp_err_t snaud0_resolution_config(const snaud0_resolution_t res, uint8_t *reg)
{
    switch (res) {
    case ES8311_RESOLUTION_16:
        *reg |= (3 << 2);
        break;
    case ES8311_RESOLUTION_18:
        *reg |= (2 << 2);
        break;
    case ES8311_RESOLUTION_20:
        *reg |= (1 << 2);
        break;
    case ES8311_RESOLUTION_24:
        *reg |= (0 << 2);
        break;
    case ES8311_RESOLUTION_32:
        *reg |= (4 << 2);
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t snaud0_fmt_config(snaud0_handle_t dev, const snaud0_resolution_t res_in, const snaud0_resolution_t res_out)
{
    uint8_t reg09 = 0; // SDP In
    uint8_t reg0a = 0; // SDP Out

    ESP_LOGI(TAG, "ES8311 in Slave mode and I2S format");
    uint8_t reg00;
    ESP_RETURN_ON_ERROR(snaud0_read_reg(dev, ES8311_RESET_REG00, &reg00), TAG, "I2C read/write error");
    reg00 &= 0xBF;
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_RESET_REG00, reg00), TAG, "I2C read/write error"); // Slave serial port - default

    /* Setup SDP In and Out resolution */
    snaud0_resolution_config(res_in, &reg09);
    snaud0_resolution_config(res_out, &reg0a);

    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_SDPIN_REG09, reg09), TAG, "I2C read/write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, ES8311_SDPOUT_REG0A, reg0a), TAG, "I2C read/write error");

    return ESP_OK;
}

esp_err_t snaud0_microphone_config(snaud0_handle_t dev, bool digital_mic)
{
    uint8_t reg14 = 0x1A; // enable analog MIC and max PGA gain

    /* PDM digital microphone enable or disable */
    if (digital_mic) {
        reg14 |= BIT(6);
    }
    snaud0_write_reg(dev, ES8311_ADC_REG17, 0xC8); // Set ADC gain @todo move this to ADC config section

    return snaud0_write_reg(dev, ES8311_SYSTEM_REG14, reg14);
}



esp_err_t snaud0_init_audio_playback(snaud0_handle_t dev)
{
    ESP_RETURN_ON_FALSE(
        (clk_cfg->sample_frequency >= 8000) && (clk_cfg->sample_frequency <= 48000),
        ESP_ERR_INVALID_ARG, TAG, "SNAUD0 init needs frequency in interval [8000; 48000] Hz"
    );
    if (!clk_cfg->mclk_from_mclk_pin) {
        ESP_RETURN_ON_FALSE(res_out == res_in, ESP_ERR_INVALID_ARG, TAG, "Resolution IN/OUT must be equal if MCLK is taken from SCK pin");
    }


    /* Reset SNAUD0 to its default */
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_SWRST_CTRL, 0x51), TAG, "I2C write error SNAUD_SWRST_CTRL");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_SWRST_CTRL, 0x00), TAG, "I2C write error SNAUD_SWRST_CTRL");

    //PLL Output Frequency Setting
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_FPLL_CTRL, 0x46), TAG, "I2C write error SNAUD_FPLL_CTRL");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_FPLL_FRA0, 0x03), TAG, "I2C write error SNAUD_FPLL_FRA0"); 
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_FPLL_FRA1, 0x074), TAG, "I2C write error SNAUD_FPLL_FRA1");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_FPLL_FRA2, 0xBC), TAG, "I2C write error SNAUD_FPLL_FRA2");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_FPLL_INT, 0x09), TAG, "I2C write error SNAUD_FPLL_INT"); 


    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_MCLK_CTRL, 0x88), TAG, "I2C write error SNAUD_MCLK_CTRL");


    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_LDO_CTRL, 0x04), TAG, "I2C write error");


    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_I2S_RX_CTRL0, 0x03), TAG, "I2C write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_I2S_RX_CTRL1, 0x01), TAG, "I2C write error");


    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_CLASS_D_PWM_CTRL0, 0x03), TAG, "I2C write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_CLASS_D_PWM_CTRL1, 0x1c), TAG, "I2C write error");
    ESP_RETURN_ON_ERROR(snaud0_write_reg(dev, SNAUD_CLASS_D_PWM_CTRL2, 0x01), TAG, "I2C write error");

    return ESP_OK;
}

void snaud0_delete(snaud0_handle_t dev)
{
    free(dev);
}

esp_err_t snaud0_voice_volume_set(snaud0_handle_t dev, int volume, int *volume_set)
{
    if (volume < 0) {
        volume = 0;
    } else if (volume > 100) {
        volume = 100;
    }

    int reg32;
    if (volume == 0) {
        reg32 = 0;
    } else {
        reg32 = ((volume) * 256 / 100) - 1;
    }

    // provide user with real volume set
    if (volume_set != NULL) {
        *volume_set = volume;
    }
    return snaud0_write_reg(dev, SNAUD0_DAC_REG32, reg32);
}

esp_err_t snaud0_voice_volume_get(snaud0_handle_t dev, int *volume)
{
    uint8_t reg32;
    ESP_RETURN_ON_ERROR(snaud0_read_reg(dev, SNAUD0_DAC_REG32, &reg32), TAG, "I2C read/write error");

    if (reg32 == 0) {
        *volume = 0;
    } else {
        *volume = ((reg32 * 100) / 256) + 1;
    }
    return ESP_OK;
}

esp_err_t snaud0_voice_mute(snaud0_handle_t dev, bool mute)
{
    uint8_t reg31;
    ESP_RETURN_ON_ERROR(snaud0_read_reg(dev, SNAUD0_DAC_REG31, &reg31), TAG, "I2C read/write error");

    if (mute) {
        reg31 |= BIT(6) | BIT(5);
    } else {
        reg31 &= ~(BIT(6) | BIT(5));
    }

    return snaud0_write_reg(dev, SNAUD0_DAC_REG31, reg31);
}

esp_err_t snaud0_microphone_gain_set(snaud0_handle_t dev, snaud0_mic_gain_t gain_db)
{
    return snaud0_write_reg(dev, SNAUD0_ADC_REG16, gain_db); // ADC gain scale up
}

esp_err_t snaud0_voice_fade(snaud0_handle_t dev, const snaud0_fade_t fade)
{
    uint8_t reg37;
    ESP_RETURN_ON_ERROR(snaud0_read_reg(dev, SNAUD0_DAC_REG37, &reg37), TAG, "I2C read/write error");
    reg37 &= 0x0F;
    reg37 |= (fade << 4);
    return snaud0_write_reg(dev, SNAUD0_DAC_REG37, reg37);
}

esp_err_t snaud0_microphone_fade(snaud0_handle_t dev, const snaud0_fade_t fade)
{
    uint8_t reg15;
    ESP_RETURN_ON_ERROR(snaud0_read_reg(dev, SNAUD0_ADC_REG15, &reg15), TAG, "I2C read/write error");
    reg15 &= 0x0F;
    reg15 |= (fade << 4);
    return snaud0_write_reg(dev, SNAUD0_ADC_REG15, reg15);
}

void snaud0_register_dump(snaud0_handle_t dev)
{
    for (int reg = 0; reg < 0x4A; reg++) {
        uint8_t value;
        ESP_ERROR_CHECK(snaud0_read_reg(dev, reg, &value));
        printf("REG:%02x: %02x", reg, value);
    }
}

snaud0_handle_t snaud0_create(const i2c_port_t port, const uint16_t dev_addr)
{
    snaud0_dev_t *sensor = (snaud0_dev_t *) calloc(1, sizeof(snaud0_dev_t));
    sensor->port = port;
    sensor->dev_addr = dev_addr;
    return (snaud0_handle_t) sensor;
}
