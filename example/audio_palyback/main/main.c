/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_system.h"
#include "esp_check.h"
#include "example_config.h"
#include "esp_log.h"
#include "audio_player.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

#include "snaud.h"

#define TAG "AUDO02_MP3"

#define CONFIG_BSP_I2S_NUM 1

/* Audio */
#define BSP_I2S_SCLK          (GPIO_NUM_17)
#define BSP_I2S_MCLK          (GPIO_NUM_2)
#define BSP_I2S_LCLK          (GPIO_NUM_47)
#define BSP_I2S_DOUT          (GPIO_NUM_15) // To Codec ES8311
#define BSP_I2S_DSIN          (GPIO_NUM_16) // From ADC ES7210
#define BSP_POWER_AMP_IO      (GPIO_NUM_46)
#define BSP_MUTE_STATUS       (GPIO_NUM_1)


#define BSP_I2S_GPIO_CFG       \
    {                          \
        .mclk = BSP_I2S_MCLK,  \
        .bclk = BSP_I2S_SCLK,  \
        .ws = BSP_I2S_LCLK,    \
        .dout = BSP_I2S_DOUT,  \
        .din = BSP_I2S_DSIN,   \
        .invert_flags = {      \
            .mclk_inv = false, \
            .bclk_inv = false, \
            .ws_inv = false,   \
        },                     \
    }


/************************aud02*************************/
#define	AUD02_DAC_DEFAULT_SETTINGS() {				\
	.eMode = eAUD02_SLAVE,							\
	.eClockSource = eAUD02_EXTERNAL_SYS_CLK,		\
    .eSamplingRate = eAUD02_44100Hz,            \
	.DAC.eAmpChannel = AMP_CH_MIX,     \
	.DAC.eAmpVol = AMP_VOL_6dB,			\
	.DAC.eAmpOpaVol = AMP_OPA_VOL_0dB,			\
	.DAC.eAmpDataOpt = DATA_OPT_0p85dB,		\
	.DAC.eHeadphoneDepop = eAUD02_DEPOP_1,			\
	.DAC.ChargeDelayMs = 2000,						\
}

#define I2S_STD_CLK_CONFIG(rate) { \
    .sample_rate_hz = rate, \
    .clk_src = I2S_CLK_SRC_DEFAULT, \
    .mclk_multiple = I2S_MCLK_MULTIPLE_256, \
}

#define I2S_CHANNEL_CONFIG(i2s_num, i2s_role) { \
    .id = i2s_num, \
    .role = i2s_role, \
    .dma_desc_num = 6, \
    .dma_frame_num = 240, \
    .auto_clear = false, \
    .intr_priority = 0, \
}
#define I2S_STD_PHILIPS_SLOT_CONFIG(bits_per_sample, mono_or_stereo) { \
    .data_bit_width = bits_per_sample, \
    .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO, \
    .slot_mode = mono_or_stereo, \
    .slot_mask = I2S_STD_SLOT_BOTH, \
    .ws_width = bits_per_sample, \
    .ws_pol = false, \
    .bit_shift = true, \
    .left_align = true, \
    .big_endian = false, \
    .bit_order_lsb = false \
}


#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg = I2S_STD_CLK_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_STD_PHILIPS_SLOT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
    }

static i2s_chan_handle_t i2s_tx_chan;
static i2s_chan_handle_t i2s_rx_chan;

extern const char mp3_start[] asm("_binary_gs_16b_1c_44100hz_mp3_start");
extern const char mp3_end[]   asm("_binary_gs_16b_1c_44100hz_mp3_end");


static esp_err_t bsp_i2s_write(void * audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = i2s_channel_write(i2s_tx_chan, (char *)audio_buffer, len, bytes_written, timeout_ms);
    return ret;
}

static esp_err_t bsp_i2s_reconfig_clk(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)bits_cfg, (i2s_slot_mode_t)ch),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };

    ret |= i2s_channel_disable(i2s_tx_chan);
    ret |= i2s_channel_reconfig_std_clock(i2s_tx_chan, &std_cfg.clk_cfg);
    ret |= i2s_channel_reconfig_std_slot(i2s_tx_chan, &std_cfg.slot_cfg);
    ret |= i2s_channel_enable(i2s_tx_chan);
    return ret;
}

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting) {
    ESP_LOGI(TAG, "mute setting %d", setting);
    return ESP_OK;
}

static esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel)
{
    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, tx_channel, rx_channel));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (tx_channel != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx_channel, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(*tx_channel));
    }
    if (rx_channel != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(*rx_channel, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(*rx_channel));
    }

    /* Setup power amplifier pin */
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(BSP_POWER_AMP_IO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    return ESP_OK;
}

static audio_player_callback_event_t expected_event;
static QueueHandle_t event_queue;

static void audio_player_callback(audio_player_cb_ctx_t *ctx)
{
    configASSERT(ctx->audio_event == expected_event);

    // wake up the test so it can continue to the next step
    configASSERT(xQueueSend(event_queue, &(ctx->audio_event), 0) == pdPASS);
}

void audio_player_init()
{
    audio_player_callback_event_t event;

    /* Configure I2S peripheral and Power Amplifier */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };
    esp_err_t ret = bsp_audio_init(&std_cfg, &i2s_tx_chan, &i2s_rx_chan);
    configASSERT(ret== ESP_OK);

    audio_player_config_t config = { .mute_fn = audio_mute_function,
                                     .write_fn = bsp_i2s_write,
                                     .clk_set_fn = bsp_i2s_reconfig_clk,
                                     .priority = 0,
                                     .coreID = 0 };
    ret = audio_player_new(config);
    configASSERT(ret == ESP_OK);

    event_queue = xQueueCreate(1, sizeof(audio_player_callback_event_t));
    configASSERT(event_queue);

    ret = audio_player_callback_register(audio_player_callback, NULL);
    configASSERT(ret== ESP_OK);

    audio_player_state_t state = audio_player_get_state();
    configASSERT(state == AUDIO_PLAYER_STATE_IDLE);



    // -1 due to the size being 1 byte too large, I think because end is the byte
    // immediately after the last byte in the memory but I'm not sure - cmm 2022-08-20
    //
    // Suppression as these are linker symbols and cppcheck doesn't know how to ensure
    // they are the same object
    // cppcheck-suppress comparePointers
    size_t mp3_size = (mp3_end - mp3_start) - 1;
    ESP_LOGI(TAG, "mp3_size %zu bytes", mp3_size);

    FILE *fp = fmemopen((void*)mp3_start, mp3_size, "rb");
    configASSERT(fp);



    ///////////////
    expected_event = AUDIO_PLAYER_CALLBACK_EVENT_PLAYING;
    ret = audio_player_play(fp);
    configASSERT(ret == ESP_OK);

    // wait for playing event to arrive
    configASSERT(xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100)) == pdPASS);

    // confirm state is playing
    state = audio_player_get_state();
    configASSERT(state == AUDIO_PLAYER_STATE_PLAYING);


    int sleep_seconds = 16;
    ESP_LOGI(TAG, "sleeping for %d seconds for playback to complete", sleep_seconds);
    vTaskDelay(pdMS_TO_TICKS(sleep_seconds * 1000));

    // wait for idle event to arrive
    configASSERT(xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100)) == pdPASS);

    state = audio_player_get_state();
    configASSERT(state == AUDIO_PLAYER_STATE_IDLE);



    ///////////////
    expected_event = AUDIO_PLAYER_CALLBACK_EVENT_SHUTDOWN;
    ret = audio_player_delete();
    configASSERT(ret == ESP_OK);

    // wait for idle event to arrive
    configASSERT(xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100)) == pdPASS);

    state = audio_player_get_state();
    configASSERT(state == AUDIO_PLAYER_STATE_SHUTDOWN);

    vQueueDelete(event_queue);

    i2s_channel_disable(i2s_tx_chan);
    i2s_channel_disable(i2s_rx_chan);
    i2s_del_channel(i2s_tx_chan);
    i2s_del_channel(i2s_rx_chan);

    ESP_LOGI(TAG, "NOTE: a memory leak will be reported the first time this test runs.\n");
    ESP_LOGI(TAG, "esp-idf v4.4.1 and v4.4.2 both leak memory between i2s_driver_install() and i2s_driver_uninstall()\n");
}


void AUD02_PLAY_Init(void)
{
    AUD02_STRUCT psCodec = AUD02_DAC_DEFAULT_SETTINGS();
    I2C_Mater_Init();
    //AUD02_List_Reg();//read test
    AUD02_DAC_Init(&psCodec);
}




void app_main(void)
{

}
