/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "app_ifft.h"
#include "app_gpio.h"
#include "esp_timer.h"
#include "app_uart.h"
#define CONFIG_LESS_INTERFERENCE_CHANNEL    40
#if CONFIG_IDF_TARGET_ESP32C5
    #define CONFIG_WIFI_BAND_MODE   WIFI_BAND_MODE_5G_ONLY
    #define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT40
    #define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT40
    #define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
    #define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C6
    #define CONFIG_WIFI_BANDWIDTH           WIFI_BW_HT40
    #define CONFIG_WIFI_PROTOCOL             WIFI_PROTOCOL_11N
#endif
#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT40
#define CONFIG_GAIN_CONTROL                 1   // 1:enable gain control, 0:disable gain control
#define CONFIG_FORCE_GAIN                   1   // 1:force gain control, 0:automatic gain control
#define CONFIG_PRINT_CSI_DATA               0
#define FORCE_AGC_GAIN                      25   
int64_t time_zero=0;
typedef struct {
    uint32_t id;
    uint32_t time;
    uint8_t fft_gain;
    uint8_t agc_gain;
    int8_t buf[256];
} csi_send_queue_t;

QueueHandle_t csi_send_queue;
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";
typedef struct
{
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned : 32; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6
    unsigned : 16; /**< reserved */
    unsigned fft_gain : 8;
    unsigned agc_gain : 8;
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
      signed : 8;  /**< reserved */
    unsigned : 24; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;

#if CONFIG_FORCE_GAIN
    /**
     * @brief Enable/disable automatic fft gain control and set its value
     * @param[in] force_en true to disable automatic fft gain control
     * @param[in] force_value forced fft gain value
     */
    extern void phy_fft_scale_force(bool force_en, uint8_t force_value);

    /**
     * @brief Enable/disable automatic gain control and set its value
     * @param[in] force_en true to disable automatic gain control
     * @param[in] force_value forced gain value
     */
    extern void phy_force_rx_gain(int force_en, int force_value);
#endif

void phy_i2c_writeReg(int8_t block, int8_t id, int8_t addr, int data);
int8_t phy_i2c_readReg(int8_t block, int8_t id, int8_t addr);
void esp_adjust_filter_bandwidth_c5(void) {
    int8_t rx_dcap0 = phy_i2c_readReg(0x67, 1, 6);
    int8_t rx_dcap1 = phy_i2c_readReg(0x67, 1, 7);
    phy_i2c_writeReg(0x67, 1, 6, 5); 
    phy_i2c_writeReg(0x67, 1, 8, 5); 
    phy_i2c_writeReg(0x67, 1, 7, 5); 
    phy_i2c_writeReg(0x67, 1, 9, 5); 
}


static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

#if CONFIG_IDF_TARGET_ESP32C5
    ESP_ERROR_CHECK(esp_wifi_start());
    
    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
        .ghz_5g = CONFIG_WIFI_5G_PROTOCOL
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));


    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
        .ghz_5g = CONFIG_WIFI_5G_BANDWIDTHS
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));

#endif  
    // ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, WIFI_PHY_RATE_MCS0_SGI));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
#if CONFIG_IDF_TARGET_ESP32C5
    if ((CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW_HT20) || (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_5G_ONLY && CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW_HT20))
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    else
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C6
    if (CONFIG_WIFI_BANDWIDTH == WIFI_BW_HT20)
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    else
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
#if CONFIG_IDF_TARGET_ESP32C5
    esp_adjust_filter_bandwidth_c5();
#endif
}

static void wifi_esp_now_init(esp_now_peer_info_t peer) 
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE, 
        .rate = WIFI_PHY_RATE_MCS0_LGI,    
        .ersu = false,                     
        .dcm = false                       
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr,&rate_config));

}
uint32_t time_pre=0;
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    int64_t start_time = esp_timer_get_time(); 
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) {
        return;
    }

    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    static int s_count = 0;

#if CONFIG_FORCE_GAIN && CONFIG_GAIN_CONTROL
    static uint16_t agc_gain_sum=0; 
    static uint16_t fft_gain_sum=0; 
    static uint8_t agc_gain_force_value=0; 
    static uint8_t fft_gain_force_value=0; 
    if (s_count<100) {
        agc_gain_sum += phy_info->agc_gain;
        fft_gain_sum += phy_info->fft_gain;
    }else if (s_count == 100) {
        agc_gain_force_value = agc_gain_sum/100;
        fft_gain_force_value = fft_gain_sum/100;
        phy_fft_scale_force(1,15);//fft_gain_force_value);
        phy_force_rx_gain(1,21);//agc_gain_force_value);
        ESP_LOGI(TAG,"fft_force %d, agc_force %d",fft_gain_force_value,agc_gain_force_value);
    }
#endif

    csi_send_queue_t *csi_send_queuedata = (csi_send_queue_t *)calloc(1, sizeof(csi_send_queue_t));
    memcpy(&(csi_send_queuedata->id), info->payload+15, sizeof(uint32_t));

    csi_send_queuedata->time = info->rx_ctrl.timestamp;
    csi_send_queuedata->agc_gain = phy_info->agc_gain;
    csi_send_queuedata->fft_gain = phy_info->fft_gain;
    // ets_printf("time:%ld  %ld\n", info->rx_ctrl.timestamp,info->rx_ctrl.timestamp-time_pre);
    // time_pre = info->rx_ctrl.timestamp;
    memset(csi_send_queuedata->buf, 0, 256);
    memcpy(csi_send_queuedata->buf + 8, info->buf, info->len);
    // xQueueOverwrite(csi_send_queue, &csi_send_queuedata);
    xQueueSend(csi_send_queue, &csi_send_queuedata, 0);
    // vTaskDelay(5 / portTICK_PERIOD_MS);
    s_count++;
            int64_t end_time = esp_timer_get_time();  // 获取结束时间
    printf("Code execution time: %lld us\n", (end_time - start_time));
#if CONFIG_PRINT_CSI_DATA
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n");
    }

    ESP_LOGI(TAG, "%s", g_buffer);

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
            s_count, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
            rx_ctrl->noise_floor, phy_info->fft_gain, phy_info->agc_gain,rx_ctrl->channel,
            rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);

    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);

    for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", info->buf[i]);
    }

    ets_printf("]\"\n");
#endif
}

static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t csi_config = {
        .enable                   = true,                           
        .acquire_csi_legacy       = false,               
        .acquire_csi_force_lltf   = false,           
        .acquire_csi_ht20         = true,                  
        .acquire_csi_ht40         = true,                  
        .acquire_csi_vht          = true,                  
        .acquire_csi_su           = false,                   
        .acquire_csi_mu           = false,                   
        .acquire_csi_dcm          = false,                  
        .acquire_csi_beamformed   = false,           
        .acquire_csi_he_stbc_mode = 2,                                                                                                                                                                                                                                                                               
        .val_scale_cfg            = false,                    
        .dump_ack_en              = false,                      
        .reserved                 = false                         
    };

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    csi_send_queue = xQueueCreate(3, sizeof(csi_send_queue_t *));

}
float complex_magnitude(Complex z) {
    return sqrt(z.real * z.real + z.imag * z.imag);
}


static void uart_send_task(void *pvParameter)
{   
    static int s_count = 0;
    csi_send_queue_t *csi_send_queue_data = NULL;
    Complex x[64];
    float cir[2]={};
    int lss = 0;
    uint32_t data_num = 0;
    uint8_t agc_gain_mode = FORCE_AGC_GAIN;
    uint8_t fft_gain_mode = 15;
    while (xQueueReceive(csi_send_queue, &csi_send_queue_data, portMAX_DELAY) == pdTRUE) {
        int64_t start_time = esp_timer_get_time(); 
        UBaseType_t queue_size = uxQueueMessagesWaiting(csi_send_queue);
#if !CONFIG_FORCE_GAIN && CONFIG_GAIN_CONTROL
        float numerator = (csi_send_queue_data->agc_gain - agc_gain_mode) + (csi_send_queue_data->fft_gain - fft_gain_mode) / 4.0f;
        float scaling_factor  = powf(10.0f, -numerator / 20.0f);
#else
        float scaling_factor = 1.0f;
#endif
        for (int i = 0; i < 64; i++) {
            x[i].real = csi_send_queue_data->buf[2 * i]; 
            x[i].imag = csi_send_queue_data->buf[2 * i + 1];
        }
        fft(x, 64, 1);
        cir[0] = complex_magnitude(x[0])*scaling_factor;
        // for (int i = 64; i < 128; i++) {
        //     x[i-64].real = csi_send_queue_data->buf[2 * i]; 
        //     x[i-64].imag = csi_send_queue_data->buf[2 * i + 1];
        // }
        // fft(x, 64, 1);
        cir[1] = complex_magnitude(x[0])*scaling_factor;

        csi_data_t data = {
            .start = {0xAA, 0x55},
            .id = csi_send_queue_data->id,
            .time_delta = csi_send_queue_data->time-time_zero,
            .cir = {cir[0], cir[1]},
            .end = {0x55, 0xAA},
        };
        uart_send_data((const char *)&data, sizeof(data));
        if (data.id-data_num!=1) {
            printf("%ld,%ld\n",data_num, data.id);
        }
        data_num = data.id;
        if (queue_size>15) {
            // printf("s:%d\n", queue_size);
        }
        int64_t end_time = esp_timer_get_time();  // 获取结束时间
    // printf("Code execution time: %lld us\n", (end_time - start_time));
#if 0
        ets_printf("CSI_DATA,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            s_count++, lss, lss, lss,
            lss,lss, lss,lss ,
            lss, lss, lss);

        ets_printf(",234,0,\"[%d", csi_send_queue_data->buf[8]);

        for (int i = 1; i < 234; i++) {
            ets_printf(",%d", csi_send_queue_data->buf[i+8]);
        }

        ets_printf("]\"\n");

        
#endif
    free(csi_send_queue_data);
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    /**
     * @breif Initialize NVS
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /**
     * @breif Initialize Wi-Fi
     */
    wifi_init();

    /**
     * @breif Initialize ESP-NOW
     *        ESP-NOW protocol see: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
     */
    esp_now_peer_info_t peer = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,    
        .encrypt   = false,   
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    wifi_esp_now_init(peer);
#endif
    init_gpio();
    init_uart();
    wifi_csi_init();
    xTaskCreate(uart_send_task, "uart_send_task", 4096, NULL, 9, NULL);

}
