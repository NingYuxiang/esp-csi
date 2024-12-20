// /* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

//    This example code is in the Public Domain (or CC0 licensed, at your option.)

//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */
// #include <stdio.h>
// #include <stdint.h>
// #include <stddef.h>
// #include <string.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "esp_log.h"
// #include "driver/spi_slave.h"
// #include "driver/gpio.h"


// #define RCV_HOST    SPI2_HOST
// #define GPIO_HANDSHAKE      2
// #define GPIO_MOSI           8
// #define GPIO_MISO           13
// #define GPIO_SCLK           9
// #define GPIO_CS             14

// //Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
// void my_post_setup_cb(spi_slave_transaction_t *trans)
// {
//     gpio_set_level(GPIO_HANDSHAKE, 1);
// }

// //Called after transaction is sent/received. We use this to set the handshake line low.
// void my_post_trans_cb(spi_slave_transaction_t *trans)
// {
//     gpio_set_level(GPIO_HANDSHAKE, 0);
// }

// //Main application
// void app_main(void)
// {
//     int n = 0;
//     esp_err_t ret;

//     //Configuration for the SPI bus
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = GPIO_MOSI,
//         .miso_io_num = GPIO_MISO,
//         .sclk_io_num = GPIO_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//     };

//     //Configuration for the SPI slave interface
//     spi_slave_interface_config_t slvcfg = {
//         .mode = 0,
//         .spics_io_num = GPIO_CS,
//         .queue_size = 3,
//         .flags = 0,
//         .post_setup_cb = my_post_setup_cb,
//         .post_trans_cb = my_post_trans_cb
//     };

//     //Configuration for the handshake line
//     gpio_config_t io_conf = {
//         .intr_type = GPIO_INTR_DISABLE,
//         .mode = GPIO_MODE_OUTPUT,
//         .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
//     };

//     //Configure handshake line as output
//     gpio_config(&io_conf);
//     //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
//     gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
//     gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
//     gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

//     //Initialize SPI slave interface
//     ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
//     assert(ret == ESP_OK);

//     WORD_ALIGNED_ATTR char sendbuf[129] = "";
//     WORD_ALIGNED_ATTR char recvbuf[129] = "";
//     memset(recvbuf, 0, 33);
//     spi_slave_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     spi_csi_send_queue = xQueueCreate(6, sizeof(spi_csi_send_queue_t));
//     spi_csi_send_queue_t spi_csi_send_queuedata = {0.0, 0.0, 0};
//     if (xQueueReceive(spi_csi_send_queue, &spi_csi_send_queuedata, portMAX_DELAY) == pdTRUE) {
//         //Clear receive buffer, set send buffer to something sane
//         memset(recvbuf, 0xA5, 129);
//         memcpy(sendbuf, &spi_csi_send_queuedata, sizeof(spi_csi_send_queuedata));

//         //Set up a transaction of 128 bytes to send/receive
//         t.length = 128 * 8;
//         t.tx_buffer = sendbuf;
//         t.rx_buffer = recvbuf;

//         ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

//         //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
//         //received data from the master. Print it.
//         printf("Received: %s\n", recvbuf);
//         n++;
//     }

// }
