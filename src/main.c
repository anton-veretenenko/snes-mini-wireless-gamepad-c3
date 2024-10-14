#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/i2c_reg.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#include "sys/time.h"
#include "esp_pm.h"

static void main_task(void *);
static void gamepads_update(void);

// static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
// #define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

static const char *TAG = "smwlgp";
static QueueHandle_t s_data_queue;
#define ESPNOW_QUEUE_SIZE 32
#define ESPNOW_CHANNEL 1
// recevier mac: 84:f7:03:d7:b4:98
const uint8_t s_mac_receiver[ESP_NOW_ETH_ALEN] = { 0x84, 0xf7, 0x03, 0xd7, 0xb4, 0x98 };
#define ESPNOW_PMK "pmk12345"
#define ESPNOW_LMK "lmk12345"
int btn_prev = 0;

#define GPIO_LED GPIO_NUM_8
#define GPIO_GAMEPAD_PWR GPIO_NUM_1

#define I2C_SCL 4
#define I2C_SDA 3
#define I2C_PORT I2C_NUM_0
#define I2C_CLOCK 200000
#define I2C_BUF_LEN 128
#define GAMEPAD_ADDR 0x52
#define GAMEPAD_BUF_LENGTH 21

#define POWERDOWN_TIMEOUT 5

bool gamepad_connected = false;
// default buf for prev button comparison, reset after deep sleep, so we use clear buttons template
static uint8_t gamepad_buf[GAMEPAD_BUF_LENGTH] = { 0x00 };
const uint8_t gamepad_clear_buttons[] = { 0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x00, 0xff, 0xff };
RTC_DATA_ATTR uint8_t gamepad_sleep_buf[sizeof(gamepad_clear_buttons)];
const uint8_t gamepad_home_btn[] = { 0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x00, 0xf7, 0xff };
const uint8_t gamepad_home_btn_send_buf[] = { 0x00, 0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x00, 0xf7, 0xff };
const uint8_t gamepad_id[] = { 0x01, 0x00, 0xa4, 0x20, 0x03, 0x01 };
volatile uint64_t gamepad_last_update = 0;

typedef struct {
    bool is_send_cb;
    esp_now_send_status_t status;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t data_len;
    uint8_t *data;
} espnow_event_t;

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel( ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

void nvs_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    espnow_event_t evt;
    memset(&evt, 0, sizeof(espnow_event_t));
    evt.is_send_cb = true;
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    memcpy(evt.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    evt.status = status;
    if (xQueueSend(s_data_queue, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

// static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
// {
//     espnow_event_t evt;
//     memset(&evt, 0, sizeof(espnow_event_t));
//     uint8_t * mac_addr = recv_info->src_addr;
//     uint8_t * des_addr = recv_info->des_addr;

//     if (mac_addr == NULL || data == NULL || len <= 0) {
//         ESP_LOGE(TAG, "Receive cb arg error");
//         return;
//     }
//     if (len > sizeof(gamepad_buf)+1) {
//         ESP_LOGE(TAG, "Receive cb, data too long");
//         return;
//     }

//     if (IS_BROADCAST_ADDR(des_addr)) {
//         /* If added a peer with encryption before, the receive packets may be
//          * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
//          * Users can check the destination address to distinguish it.
//          */
//         ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
//     } else {
//         ESP_LOGD(TAG, "Receive unicast ESPNOW data");
//     }

//     evt.is_send_cb = false;
//     memcpy(evt.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
//     evt.data = malloc(len);
//     if (evt.data == NULL) {
//         ESP_LOGE(TAG, "Malloc receive data fail");
//         return;
//     }
//     memcpy(evt.data, data, len);
//     evt.data_len = len;
//     if (xQueueSend(s_data_queue, &evt, 0) != pdTRUE) {
//         ESP_LOGW(TAG, "Send receive queue fail");
//         free(evt.data);
//     }
// }

static esp_err_t espnow_init(void)
{
    s_data_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (s_data_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    // ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    // ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    // ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)ESPNOW_PMK) );

    /* add receiver peer */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_data_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->lmk, ESPNOW_LMK, sizeof(ESPNOW_LMK));
    // add receiver peer
    memcpy(peer->peer_addr, s_mac_receiver, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    return ESP_OK;
}

static void espnow_deinit(void)
{
    vSemaphoreDelete(s_data_queue);
    esp_now_deinit();
}

static void gamepads_update(void)
{
    static uint8_t init_buf1[2] = { 0xF0, 0x55 };
    static uint8_t init_buf2[2] = { 0xFB, 0x00 };
    static uint8_t init_buf3[2] = { 0xFE, 0x03 };
    static uint8_t btn_read_cmd[1] = { 0x00 };
    static uint8_t btn_buf[sizeof(gamepad_buf)] = { 0x00 };
    static uint8_t send_buf[sizeof(gamepad_buf)+1];
    esp_err_t err;
    struct timeval tv;
    
    if (!gamepad_connected) {
        err = i2c_master_write_to_device(
            I2C_PORT, GAMEPAD_ADDR,
            init_buf1, sizeof(init_buf1),
            8 / portTICK_PERIOD_MS
        );
        ESP_LOGI(TAG, "I2C init ret 1: %d", err);
        // if (err == ESP_FAIL) {
        //     esp_wifi_start();
        //     esp_now_send(s_mac_receiver, gamepad_home_btn_send_buf, sizeof(gamepad_home_btn_send_buf));
        // }

        if (err == ESP_OK) {
            err = i2c_master_write_to_device(
                I2C_PORT, GAMEPAD_ADDR,
                init_buf2, sizeof(init_buf2),
                8 / portTICK_PERIOD_MS
            );
            ESP_LOGI(TAG, "I2C init ret 2: %d", err);

            if (err == ESP_OK) {
                err = i2c_master_write_to_device(
                    I2C_PORT, GAMEPAD_ADDR,
                    init_buf3, sizeof(init_buf3),
                    8 / portTICK_PERIOD_MS
                );
                ESP_LOGI(TAG, "I2C init ret 3: %d", err);

                if (err == ESP_OK) {
                    gamepad_connected = true;
                    uint8_t read_id_cmd[] = { 0xfa };
                    err = i2c_master_write_to_device(
                        I2C_PORT, GAMEPAD_ADDR,
                        read_id_cmd, sizeof(read_id_cmd),
                        8 / portTICK_PERIOD_MS
                    );
                    err = i2c_master_read_from_device(
                        I2C_PORT, GAMEPAD_ADDR,
                        btn_buf, 6,
                        8 / portTICK_PERIOD_MS
                    );
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "Gamepad ID: %02x %02x %02x %02x %02x %02x",
                            btn_buf[0], btn_buf[1], btn_buf[2], btn_buf[3], btn_buf[4], btn_buf[5]
                        );
                        // force read buttons after id read
                        err = i2c_master_write_to_device(
                            I2C_PORT, GAMEPAD_ADDR,
                            btn_read_cmd, sizeof(btn_read_cmd),
                            8 / portTICK_PERIOD_MS
                        );
                        err = i2c_master_read_from_device(
                            I2C_PORT, GAMEPAD_ADDR,
                            btn_buf, sizeof(btn_buf),
                            8 / portTICK_PERIOD_MS
                        );
                    } else {
                        ESP_LOGE(TAG, "Gamepad: read id err %d", err);
                    }
                } else {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            } else {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    if (gamepad_connected) {
        // read gamepad buttons
        vTaskDelay(30 / portTICK_PERIOD_MS);
        err = i2c_master_write_to_device(
            I2C_PORT, GAMEPAD_ADDR,
            btn_read_cmd, sizeof(btn_read_cmd),
            8 / portTICK_PERIOD_MS
        );
        err = i2c_master_read_from_device(
            I2C_PORT, GAMEPAD_ADDR,
            btn_buf, sizeof(btn_buf),
            8 / portTICK_PERIOD_MS
        );
        if (err == ESP_OK) {
            // print buttons buf
            if (memcmp(btn_buf, gamepad_buf, sizeof(btn_buf)) != 0) {
                // send new data
                esp_wifi_start();
                memcpy(gamepad_buf, btn_buf, sizeof(gamepad_buf));
                ESP_LOGI(TAG, "Left buttons: %02x %02x %02x %02x %02x %02x %02x %02x",
                    gamepad_buf[0], gamepad_buf[1], gamepad_buf[2], gamepad_buf[3],
                    gamepad_buf[4], gamepad_buf[5], gamepad_buf[6], gamepad_buf[7]
                );
                // printf("Left buttons: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                //     gamepad_left_buf[8], gamepad_left_buf[9], gamepad_left_buf[10], gamepad_left_buf[11],
                //     gamepad_left_buf[12], gamepad_left_buf[13], gamepad_left_buf[14], gamepad_left_buf[15]
                // );
                // printf("Left buttons: %02x %02x %02x %02x %02x\n",
                //     gamepad_left_buf[16], gamepad_left_buf[17], gamepad_left_buf[18], gamepad_left_buf[19],
                //     gamepad_left_buf[20]
                // );
                send_buf[0] = 0x00; // left gamepad // TODO: add left/right hardware config
                memcpy(send_buf+1, gamepad_buf, sizeof(gamepad_buf));
                esp_now_send(s_mac_receiver, send_buf, sizeof(send_buf));
                gettimeofday(&tv, NULL);
                gamepad_last_update = tv.tv_sec;
                ESP_LOGI(TAG, "Buttons update");
            }
        } else {
            ESP_LOGE(TAG, "Gamepad: read err %d", err);
            gamepad_connected = false;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

static void gpio_init(void)
{
    // led
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << GPIO_LED;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // // button
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.pin_bit_mask = 1ULL << GPIO_BTN;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_up_en = 1;
    // gpio_config(&io_conf);

    // gamepad power
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1ULL << GPIO_GAMEPAD_PWR;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_GAMEPAD_PWR, 1); // enable gamepad power
    gpio_hold_en(GPIO_GAMEPAD_PWR); // hold pin value during light sleep
}

static void i2c_init_gamepads(void)
{
    esp_err_t err;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_CLOCK
    };
    err = i2c_param_config(I2C_PORT, &conf);
    if (err == ESP_OK) {
        int timeout = 0;
        i2c_get_timeout(I2C_PORT, &timeout);
        ESP_LOGI(TAG, "I2C timeout: %d", timeout);
        i2c_set_timeout(I2C_PORT, 9320000);
        i2c_get_timeout(I2C_PORT, &timeout);
        ESP_LOGI(TAG, "I2C timeout set: %d", timeout);

        ESP_ERROR_CHECK( i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0) );
        ESP_LOGI(TAG, "I2C init done");
    } else {
        ESP_LOGE(TAG, "I2C init err %d", err);
    }
}

static void sleep(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec - gamepad_last_update > POWERDOWN_TIMEOUT * 60) {
        ESP_LOGI(TAG, "Going to sleep at %lld", tv.tv_sec);
        gpio_hold_dis(GPIO_GAMEPAD_PWR);
        gpio_set_level(GPIO_GAMEPAD_PWR, 0); // disable gamepad power
        ESP_ERROR_CHECK( esp_sleep_enable_timer_wakeup(30 * 1000 * 1000) );
        esp_sleep_config_gpio_isolate();
        espnow_deinit();
        esp_wifi_stop();
        nvs_flash_deinit();
        memcpy(gamepad_sleep_buf, gamepad_buf, sizeof(gamepad_sleep_buf));
        esp_deep_sleep_start();
        // sleeping for 30 seconds until RESET !!
        // gettimeofday(&tv, NULL);
        // ESP_LOGI(TAG, "Woke up at %lld", tv.tv_sec);
        // nvs_init();
        // wifi_init();
        // espnow_init();
    } else {
        // light sleep for 10ms
        // esp_sleep_config_gpio_isolate();
        // ESP_ERROR_CHECK( esp_sleep_enable_timer_wakeup(10 * 1000 * 1000) );
        // ESP_ERROR_CHECK( esp_light_sleep_start() );
        // espnow_deinit();
        // esp_wifi_stop();
        // vTaskDelay(40 / portTICK_PERIOD_MS);
        // esp_wifi_start();
        // espnow_init();
    }
}

static void main_task(void *)
{
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Task started");

    memcpy(gamepad_buf, gamepad_sleep_buf, sizeof(gamepad_sleep_buf));
    i2c_init_gamepads();
    
    esp_wifi_stop();
    while (true) {
        gamepads_update(); // will enable wifi if buttons change
        sleep();
    }
}

static void espnow_send_cb_task(void *)
{
    espnow_event_t evt;

    while (true) {
        if (xQueueReceive(s_data_queue, &evt, 100 / portTICK_PERIOD_MS) == pdTRUE) {
            if (evt.is_send_cb) {
                ESP_LOGI(TAG, "Sent data to "MACSTR", status: %d, data_len: %d", MAC2STR(evt.mac_addr), evt.status, evt.data_len);
            }
            if (evt.data_len > 0) {
                free(evt.data);
            }
            esp_wifi_stop(); // stop wifi until next button change
        }
    }
}

void power_init(void)
{
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 10,
        .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);
}

void app_main(void)
{  
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    power_init();
    esp_deep_sleep_disable_rom_logging();
    gpio_init();

    nvs_init();
    wifi_init();
    ESP_ERROR_CHECK( espnow_init() );

    xTaskCreate(main_task, "main_task", 10240, NULL, 4, NULL);
    xTaskCreate(espnow_send_cb_task, "send_cb_task", 4096, NULL, 4, NULL);

    // blink start led
    while (true) {
        gpio_set_level(GPIO_LED, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_LED, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}