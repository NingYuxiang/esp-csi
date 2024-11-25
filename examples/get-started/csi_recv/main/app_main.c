/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

#define CONFIG_LESS_INTERFERENCE_CHANNEL    11
#if CONFIG_IDF_TARGET_ESP32C5
    #define CONFIG_WIFI_BAND_MODE   WIFI_BAND_MODE_2G_ONLY
    #define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT40
    #define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT40
    #define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
    #define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C6
    #define CONFIG_WIFI_BANDWIDTH           WIFI_BW_HT40
    #define CONFIG_WIFI_PROTOCOL             WIFI_PROTOCOL_11N
#endif
#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT40
#define CONFIG_GAIN_CONTROL                 1
#define CONFIG_FORCE_GAIN                   0

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

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) {
        return;
    }

    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
    static int s_count = 0;
#if CONFIG_GAIN_CONTROL
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
    #if CONFIG_FORCE_GAIN
        phy_fft_scale_force(1,fft_gain_force_value);
        phy_force_rx_gain(1,agc_gain_force_value);
    #endif
        ESP_LOGW(TAG,"fft_force %d, agc_force %d",fft_gain_force_value,agc_gain_force_value);
    }
#endif

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
            s_count++, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
            rx_ctrl->noise_floor, phy_info->fft_gain, phy_info->agc_gain,rx_ctrl->channel,
            rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
#else
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            s_count++, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
            rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
            rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
            rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
            rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);

#endif

    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);

    for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", info->buf[i]);
    }

    ets_printf("]\"\n");
}

static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(g_wifi_radar_config->wifi_sniffer_cb));

    /**< default config */
#if CONFIG_IDF_TARGET_ESP32C5
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
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,                           
        .acquire_csi_legacy     = true,                        
        .acquire_csi_ht20       = true,                  
        .acquire_csi_ht40       = true,                                   
        .acquire_csi_su         = true,                   
        .acquire_csi_mu         = true,                   
        .acquire_csi_dcm        = true,                  
        .acquire_csi_beamformed = true,           
        .acquire_csi_he_stbc    = 2,                                                                                                                                                                                                                                                                               
        .val_scale_cfg          = false,                    
        .dump_ack_en            = true,                      
        .reserved               = false                         
    };
#else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
#endif
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
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
    wifi_csi_init();
}
