#pragma once

#include "esp_types.h"
#include "esp_err.h"

#include "driver/i2c.h"

/* SNAUD0 address: CE pin low - 0x18, CE pin high - 0x19 */
#define SNAUD0_ADDRRES_0 0x32

#ifdef __cplusplus
extern "C" {
#endif

typedef void *snaud0_handle_t;

typedef enum {
    SNAUD0_MIC_GAIN_MIN = -1,
    SNAUD0_MIC_GAIN_0DB,
    SNAUD0_MIC_GAIN_6DB,
    SNAUD0_MIC_GAIN_12DB,
    SNAUD0_MIC_GAIN_18DB,
    SNAUD0_MIC_GAIN_24DB,
    SNAUD0_MIC_GAIN_30DB,
    SNAUD0_MIC_GAIN_36DB,
    SNAUD0_MIC_GAIN_42DB,
    SNAUD0_MIC_GAIN_MAX
} snaud0_mic_gain_t;

typedef enum {
    SNAUD0_FADE_OFF = 0,
    SNAUD0_FADE_4LRCK, // 4LRCK means ramp 0.25dB/4LRCK
    SNAUD0_FADE_8LRCK,
    SNAUD0_FADE_16LRCK,
    SNAUD0_FADE_32LRCK,
    SNAUD0_FADE_64LRCK,
    SNAUD0_FADE_128LRCK,
    SNAUD0_FADE_256LRCK,
    SNAUD0_FADE_512LRCK,
    SNAUD0_FADE_1024LRCK,
    SNAUD0_FADE_2048LRCK,
    SNAUD0_FADE_4096LRCK,
    SNAUD0_FADE_8192LRCK,
    SNAUD0_FADE_16384LRCK,
    SNAUD0_FADE_32768LRCK,
    SNAUD0_FADE_65536LRCK
} snaud0_fade_t;

typedef enum snaud0_resolution_t {
    SNAUD0_RESOLUTION_16 = 16,
    SNAUD0_RESOLUTION_18 = 18,
    SNAUD0_RESOLUTION_20 = 20,
    SNAUD0_RESOLUTION_24 = 24,
    SNAUD0_RESOLUTION_32 = 32
} snaud0_resolution_t;

typedef struct snaud0_clock_config_t {
    bool mclk_inverted;
    bool sclk_inverted;
    bool mclk_from_mclk_pin; // true: from MCLK pin (pin no. 2), false: from SCLK pin (pin no. 6)
    int  mclk_frequency;     // This parameter is ignored if MCLK is taken from SCLK pin
    int  sample_frequency;   // in Hz
} snaud0_clock_config_t;

/**
 * @brief Initialize SNAUD0
 *
 * There are two ways of providing Master Clock (MCLK) signal to SNAUD0 in Slave Mode:
 * 1. From MCLK pin:
 *    For flexible scenarios. A clock signal from I2S master is routed to MCLK pin.
 *    Its frequency must be defined in clk_cfg->mclk_frequency parameter.
 * 2. From SCLK pin:
 *    For simpler scenarios. SNAUD0 takes its clock from SCK pin. MCLK pin does not have to be connected.
 *    In this case, res_in must equal res_out; clk_cfg->mclk_frequency parameter is ignored
 *    and MCLK is calculated as MCLK = clk_cfg->sample_frequency * res_out * 2.
 *    Not all sampling frequencies are supported in this mode.
 *
 * @param dev SNAUD0 handle
 * @param[in] clk_cfg Clock configuration
 * @param[in] res_in  Input serial port resolution
 * @param[in] res_out Output serial port resolution
 * @return
 *     - ESP_OK success
 *     - ESP_ERR_INVALID_ARG Sample frequency or resolution invalid
 *     - Else fail
 */
esp_err_t snaud0_init(snaud0_handle_t dev, const snaud0_clock_config_t *const clk_cfg, const snaud0_resolution_t res_in,
                      const snaud0_resolution_t res_out);

/**
 * @brief Set output volume
 *
 * Volume paramter out of <0, 100> interval will be truncated.
 *
 * @param dev SNAUD0 handle
 * @param[in] volume Set volume (0 ~ 100)
 * @param[out] volume_set Volume that was set. Same as volume, unless volume is outside of <0, 100> interval.
 *                        This parameter can be set to NULL, if user does not need this information.
 *
 * @return
 *     - ESP_OK success
 *     - Else fail
 */
esp_err_t snaud0_voice_volume_set(snaud0_handle_t dev, int volume, int *volume_set);

/**
 * @brief Get output volume
 *
 * @param dev SNAUD0 handle
 * @param[out] volume get volume (0 ~ 100)
 *
 * @return
 *     - ESP_OK success
 *     - Else fail
 */
esp_err_t snaud0_voice_volume_get(snaud0_handle_t dev, int *volume);

/**
 * @brief Print out SNAUD0 register content
 *
 * @param dev SNAUD0 handle
 */
void snaud0_register_dump(snaud0_handle_t dev);

/**
 * @brief Mute SNAUD0 output
 *
 * @param dev SNAUD0 handle
 * @param[in] enable true: mute, false: don't mute
 * @return
 *     - ESP_OK success
 *     - Else fail
 */
esp_err_t snaud0_voice_mute(snaud0_handle_t dev, bool enable);

/**
 * @brief Set Microphone gain
 *
 * @param dev SNAUD0 handle
 * @param[in] gain_db Microphone gain
 * @return
 *     - ESP_OK success
 *     - Else fail
 */
esp_err_t snaud0_microphone_gain_set(snaud0_handle_t dev, snaud0_mic_gain_t gain_db);

/**
 * @brief Configure microphone
 *
 * @param dev SNAUD0 handle
 * @param[in] digital_mic Set to true for digital microphone
 * @return
 *     - ESP_OK success
 *     - Else fail
 */
esp_err_t snaud0_microphone_config(snaud0_handle_t dev, bool digital_mic);

/**
 * @brief Configure sampling frequency
 *
 * @note This function is called by snaud0_init().
 *       Call this function explicitly only if you want to change sample frequency during runtime.
 * @param dev SNAUD0 handle
 * @param[in] mclk_frequency   MCLK frequency in [Hz] (MCLK or SCLK pin, depending on bit register01[7])
 * @param[in] sample_frequency Required sample frequency in [Hz], e.g. 44100, 22050...
 * @return
 *     - ESP_OK success
 *     - ESP_ERR_INVALID_ARG cannot set clock dividers for given MCLK and sampling frequency
 *     - Else I2C read/write error
 */
esp_err_t snaud0_sample_frequency_config(snaud0_handle_t dev, int mclk_frequency, int sample_frequency);

/**
 * @brief Configure fade in/out for ADC: voice
 *
 * @param dev SNAUD0 handle
 * @param[in] fade Fade ramp rate
 * @return
 *     - ESP_OK success
 *     - Else I2C read/write error
 */
esp_err_t snaud0_voice_fade(snaud0_handle_t dev, const snaud0_fade_t fade);

/**
 * @brief Configure fade in/out for DAC: microphone
 *
 * @param dev SNAUD0 handle
 * @param[in] fade Fade ramp rate
 * @return
 *     - ESP_OK success
 *     - Else I2C read/write error
 */
esp_err_t snaud0_microphone_fade(snaud0_handle_t dev, const snaud0_fade_t fade);

/**
 * @brief Create SNAUD0 object and return its handle
 *
 * @param[in] port     I2C port number
 * @param[in] dev_addr I2C device address of SNAUD0
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
snaud0_handle_t snaud0_create(const i2c_port_t port, const uint16_t dev_addr);

/**
 * @brief Delete SNAUD0 object
 *
 * @param dev SNAUD0 handle
 */
void snaud0_delete(snaud0_handle_t dev);

#ifdef __cplusplus
}
#endif
