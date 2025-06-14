/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zb_sensor.h"

#include <led_strip.h>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

#include "sdkconfig.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_SENSOR";

/********************* Define functions **************************/

// test firmware - start

#include "driver/uart.h"
#include "driver/gpio.h"

static void ld2450_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = LD2450_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LD2450_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LD2450_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LD2450_UART_NUM, LD2450_TX_PIN, LD2450_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "LD2450 UART initialized");
}

static bool ld2450_enter_config_mode(void)
{
    uint8_t enable_cmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t response[64];
    int len;

    ESP_LOGI(TAG, "Entering LD2450 configuration mode...");

    uart_write_bytes(LD2450_UART_NUM, (const char*)enable_cmd, sizeof(enable_cmd));
    uart_flush_input(LD2450_UART_NUM);

    vTaskDelay(pdMS_TO_TICKS(100));

    len = uart_read_bytes(LD2450_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(100));
    if (len > 0) {
        ESP_LOGI(TAG, "Config mode response (%d bytes):", len);
        for (int i = 0; i < len; i++) {
            printf("%02x ", response[i]);
        }
        printf("\n");
        return true;
    } else {
        ESP_LOGW(TAG, "No response to config mode command");
        return false;
    }
}

static bool ld2450_exit_config_mode(void)
{
    uint8_t end_cmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t response[64];
    int len;

    ESP_LOGI(TAG, "Exiting LD2450 configuration mode...");

    uart_write_bytes(LD2450_UART_NUM, (const char*)end_cmd, sizeof(end_cmd));
    vTaskDelay(pdMS_TO_TICKS(100));

    len = uart_read_bytes(LD2450_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(100));
    if (len > 0) {
        ESP_LOGI(TAG, "Exit config mode response (%d bytes):", len);
        for (int i = 0; i < len; i++) {
            printf("%02x ", response[i]);
        }
        printf("\n");
        return true;
    } else {
        ESP_LOGW(TAG, "No response to exit config mode command");
        return false;
    }
}

static bool ld2450_read_firmware_version(char *version_str, size_t max_len)
{
    uint8_t fw_cmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
    uint8_t response[64];
    int len;
    bool success = false;

    ESP_LOGI(TAG, "Reading LD2450 firmware version...");

    uart_write_bytes(LD2450_UART_NUM, (const char*)fw_cmd, sizeof(fw_cmd));
    vTaskDelay(pdMS_TO_TICKS(100));

    len = uart_read_bytes(LD2450_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Firmware response received: %d bytes", len);

    if (len > 0) {
        ESP_LOGD(TAG, "Full firmware response dump:");
        for (int i = 0; i < len; i++) {
            printf("%02x ", response[i]);
            if ((i + 1) % 16 == 0) printf("\n");
        }
        printf("\n");

        if (len >= 18) {
            uint8_t major = response[13];
            uint8_t minor = response[12];
            char build[16];
            snprintf(build, sizeof(build), "%02x%02x%02x%02x",
                    response[17], response[16], response[15], response[14]);

            snprintf(version_str, max_len, "%d.%02d.%s", major, minor, build);
            ESP_LOGI(TAG, "LD2450 Firmware Version: %s", version_str);
            success = true;
        } else {
            ESP_LOGW(TAG, "Response too short for firmware version parsing (need 18+ bytes, got %d)", len);
        }
    } else {
        ESP_LOGW(TAG, "No response to firmware version request");
    }

    return success;
}

static void report_attribute_to_coordinator(uint16_t attr_id)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = { 0 };
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_attr_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000,
    report_attr_cmd.attributeID = attr_id;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = CUSTOM_CLUSTER_ID;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send report attribute command: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Report attribute command sent successfully");
    }
}

static void ld2450_update_zigbee_fw_version_attr(const char *version_str)
{
    if (version_str == NULL) {
        ESP_LOGW(TAG, "Invalid firmware version string");
        return;
    }

    ESP_LOGI(TAG, "Updating Zigbee attribute with firmware version: %s", version_str);

    // prepare firmware version data
    uint8_t data[32];
    size_t len = strlen(version_str);
    if (len > sizeof(data) - 1) {
        len = sizeof(data) - 1;
    }

    data[0] = len;
    memcpy(&data[1], version_str, len);

    // update attr
    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        LD2450_VERSION_ATTR_ID,
        data,
        false
    );

    report_attribute_to_coordinator(LD2450_VERSION_ATTR_ID);
}

static void ld2450_get_fw_version_and_update_zigbee(void)
{
    char version_str[32] = {0};

    if (ld2450_enter_config_mode()) {
        if (ld2450_read_firmware_version(version_str, sizeof(version_str))) {
            ld2450_update_zigbee_fw_version_attr(version_str);
        }
        ld2450_exit_config_mode();
    }
}

// test firmware - end

// test read data - start


#define INVALID_COORDINATE_VALUE 0x7FFF

typedef struct {
    int16_t x[3];
    int16_t y[3];
    int16_t speed[3];
} radar_data_t;

static radar_data_t latest_radar_data = {
    .x = {INVALID_COORDINATE_VALUE, INVALID_COORDINATE_VALUE, INVALID_COORDINATE_VALUE},
    .y = {INVALID_COORDINATE_VALUE, INVALID_COORDINATE_VALUE, INVALID_COORDINATE_VALUE},
    .speed = {INVALID_COORDINATE_VALUE, INVALID_COORDINATE_VALUE, INVALID_COORDINATE_VALUE}
};

#define TARGET_BASE_ATTR_ID 0x05

#define TARGETS_COUNT 3
#define ATTRS_PER_TARGET 3

#define GENERATE_ATTR(target_idx, field, offset) \
    {TARGET_BASE_ATTR_ID + (target_idx) * ATTRS_PER_TARGET + offset, \
    {ESP_ZB_ZCL_ATTR_TYPE_S16, sizeof(int16_t), &latest_radar_data.field[target_idx]}}

#define TARGET_ATTRS(GEN_MACRO, target_idx) \
    GEN_MACRO(target_idx, x, 0), \
    GEN_MACRO(target_idx, y, 1), \
    GEN_MACRO(target_idx, speed, 2)

static TimerHandle_t zigbee_update_timer = NULL;

static SemaphoreHandle_t radar_data_mutex;

static void report_attributes_to_coordinator()
{
    esp_zb_zcl_attribute_t attr_field[] =
    {
        TARGET_ATTRS(GENERATE_ATTR, 0),
        TARGET_ATTRS(GENERATE_ATTR, 1),
        TARGET_ATTRS(GENERATE_ATTR, 2)
     };

    esp_zb_zcl_write_attr_cmd_t write_attr_cmd = { 0 };
    write_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    write_attr_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    write_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    write_attr_cmd.clusterID = CUSTOM_CLUSTER_ID;
    write_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

    write_attr_cmd.attr_number = TARGETS_COUNT * ATTRS_PER_TARGET;
    write_attr_cmd.attr_field = attr_field;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_write_attr_cmd_req(&write_attr_cmd);
    esp_zb_lock_release();
}

static void zigbee_update_timer_callback(TimerHandle_t xTimer)
{
    if (xSemaphoreTake(radar_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        report_attributes_to_coordinator();
        xSemaphoreGive(radar_data_mutex);
    }
}

#define LD2450_FRAME_SIZE 30
#define LD2450_HEADER_SIZE 4

static const uint8_t RADAR_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t RADAR_TAIL[2] = {0x55, 0xCC};

static void ld2450_read_task(void *pvParameters) {
    uint8_t rx_buffer[LD2450_FRAME_SIZE];

    while (1) {
        uint8_t byte;
        while (uart_read_bytes(LD2450_UART_NUM, &byte, 1, pdMS_TO_TICKS(100)) == 1) {
            if (byte == RADAR_HEADER[0]) {
                rx_buffer[0] = byte;

                if (uart_read_bytes(LD2450_UART_NUM, &rx_buffer[1], LD2450_HEADER_SIZE - 1, pdMS_TO_TICKS(50)) == LD2450_HEADER_SIZE - 1) {
                    if (memcmp(rx_buffer, RADAR_HEADER, LD2450_HEADER_SIZE) == 0) {
                        if (uart_read_bytes(LD2450_UART_NUM, &rx_buffer[LD2450_HEADER_SIZE],
                            LD2450_FRAME_SIZE - LD2450_HEADER_SIZE, pdMS_TO_TICKS(100)) == LD2450_FRAME_SIZE - LD2450_HEADER_SIZE) {

                            if (memcmp(&rx_buffer[LD2450_FRAME_SIZE - 2], RADAR_TAIL, 2) == 0) {
                                if (xSemaphoreTake(radar_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                    for (int target = 0; target < 3; target++) {
                                        uint8_t *target_data = &rx_buffer[4 + target * 8];

                                        bool has_data = false;
                                        for (int i = 0; i < 8; i++) {
                                            if (target_data[i] != 0) {
                                                has_data = true;
                                                break;
                                            }
                                        }

                                        if (has_data) {
                                            uint16_t raw_x = (target_data[1] << 8) | target_data[0];
                                            uint16_t raw_y = (target_data[3] << 8) | target_data[2];
                                            uint16_t raw_speed = (target_data[5] << 8) | target_data[4];

                                            int16_t x;
                                            if (raw_x & 0x8000) {
                                                x = (int16_t)(raw_x & 0x7FFF);
                                            } else {
                                                x = (int16_t)(-(int)(raw_x & 0x7FFF));
                                            }

                                            int16_t y;
                                            if (raw_y & 0x8000) {
                                                y = (int16_t)(raw_y & 0x7FFF);
                                            } else {
                                                y = (int16_t)(-(int)(raw_y & 0x7FFF));
                                            }

                                            int16_t speed;
                                            if (raw_speed & 0x8000) {
                                                speed = (int16_t)(raw_speed & 0x7FFF);
                                            } else {
                                                speed = (int16_t)(-(int)(raw_speed & 0x7FFF));
                                            }

                                            latest_radar_data.x[target] = x;
                                            latest_radar_data.y[target] = y;
                                            latest_radar_data.speed[target] = speed;
                                        } else {
                                            latest_radar_data.x[target] = INVALID_COORDINATE_VALUE;
                                            latest_radar_data.y[target] = INVALID_COORDINATE_VALUE;
                                            latest_radar_data.speed[target] = INVALID_COORDINATE_VALUE;
                                        }
                                    }

                                    xSemaphoreGive(radar_data_mutex);
                                }
                                ESP_LOGI(TAG, "------------------------");
                            }
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#define THROTTLE_MS 1000

void start_radar_processing(void) {
    zigbee_update_timer = xTimerCreate(
        "ZigbeeUpdateTimer",
        pdMS_TO_TICKS(THROTTLE_MS),
        pdTRUE,
        NULL,
        zigbee_update_timer_callback
    );

    if (zigbee_update_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create Zigbee update timer");
        return;
    }

    xTimerStart(zigbee_update_timer, 0);

    xTaskCreate(ld2450_read_task, "radar_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Radar data reading is started");
}

/*
static void ld2450_read_task(void *pvParameters) {
    uint8_t rx_buffer[LD2450_FRAME_SIZE];

    while (1) {
        uint8_t byte;
        while (uart_read_bytes(LD2450_UART_NUM, &byte, 1, pdMS_TO_TICKS(100)) == 1) {
            if (byte == RADAR_HEADER[0]) {
                rx_buffer[0] = byte;

                if (uart_read_bytes(LD2450_UART_NUM, &rx_buffer[1], LD2450_HEADER_SIZE - 1, pdMS_TO_TICKS(50)) == LD2450_HEADER_SIZE - 1) {
                    if (memcmp(rx_buffer, RADAR_HEADER, LD2450_HEADER_SIZE) == 0) {
                        if (uart_read_bytes(LD2450_UART_NUM, &rx_buffer[LD2450_HEADER_SIZE],
                            LD2450_FRAME_SIZE - LD2450_HEADER_SIZE, pdMS_TO_TICKS(100)) == LD2450_FRAME_SIZE - LD2450_HEADER_SIZE) {

                            if (memcmp(&rx_buffer[LD2450_FRAME_SIZE - 2], RADAR_TAIL, 2) == 0) {
                                ESP_LOGI(TAG, "Radar data frame:");
                                for (int i = 0; i < LD2450_FRAME_SIZE; i++) {
                                    printf("%02X ", rx_buffer[i]);
                                }
                                printf("\n");
                            }
                            }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}*/

/*void start_radar_processing(void) {
    xTaskCreate(ld2450_read_task, "radar_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Radar data reading is started");
}*/

// test read data - end

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited) {
        light_driver_init(LIGHT_DEFAULT_OFF);
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_SENSOR_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
                light_driver_set_color_RGB(255, 255, 0);
                light_driver_set_power(light_state);
            }
        } else if (message->info.cluster == CUSTOM_CLUSTER_ID) {
            /*for (int i = 0; i < message->attribute.data.size; i++) {
                ESP_LOGI(TAG, "0x%2x ", *(uint8_t *)(message->attribute.data.value + i));
            }
            ESP_LOGI(TAG, "\n");*/
            if (message->attribute.data.value != NULL && message->attribute.data.size > 0) {
                if (message->attribute.id == 0x01) {
                    uint8_t color_value = *(uint8_t *)(message->attribute.data.value);
                    light_driver_set_color_RGB(color_value, 255, 0);
                    light_driver_set_power(true);
                }
                else {
                    ESP_LOGW(TAG, "Invalid attribute id");
                }
            } else {
                ESP_LOGW(TAG, "Custom cluster data value is NULL or empty");
            }
        }
    }
    return ret;
}

static esp_err_t zb_custom_req_handler(const esp_zb_zcl_custom_cluster_command_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_custom_cluster_cmd_resp_t req = {
        .zcl_basic_cmd.dst_addr_u.addr_short = message->info.src_address.u.short_addr,
        .zcl_basic_cmd.src_endpoint = message->info.dst_endpoint,
        .zcl_basic_cmd.dst_endpoint = message->info.src_endpoint,
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id = CUSTOM_CLUSTER_ID,
        .custom_cmd_id = message->info.command.id,
        .direction = message->info.command.direction,
        .data = {0, 0, 0},
    };

    ESP_LOGW(TAG, "Receive(%d) request: ", message->data.size);
    for (int i = 0; i < message->data.size; i++) {
        ESP_LOGW(TAG, "%02x, ", *((uint8_t *)message->data.value + i));
    }
    printf("\n");
    if (message->info.command.id == 0x01) {
        esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT, CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 0x03, message->data.value, false);
        req.data.type = ESP_ZB_ZCL_ATTR_TYPE_32BIT_ARRAY;
        req.data.value = message->data.value;
        esp_zb_zcl_custom_cluster_cmd_resp(&req);
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
        ret = zb_custom_req_handler((esp_zb_zcl_custom_cluster_command_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

signed int esp_zb_zcl_cluster_check_value_handler(uint16_t attr_id, uint8_t endpoint, uint8_t *value)
{
    //ESP_LOGW(TAG, "check value endpoint:%d, attr: %d\n", endpoint, attr_id);
    return 0;
}

void esp_zb_zcl_cluster_write_attr_handler(uint8_t endpoint, uint16_t attr_id, uint8_t *new_value, uint16_t manuf_code)
{
    //ESP_LOGW(TAG, "write attr endpoint:%d, attr: %d\n", endpoint, attr_id);
}

#define CUSTOM_STRING_MAX_SIZE 255

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //
    esp_zb_attribute_list_t *custom_attr = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    uint8_t custom_value = 0xf0;
    uint8_t custom_string[CUSTOM_STRING_MAX_SIZE] = "_ESPRESSIF";
    custom_string[0] = CUSTOM_STRING_MAX_SIZE - 1;
    uint32_t custom_array[] = {1, 2, 3, 4, 5};
    esp_zb_custom_cluster_add_custom_attr(custom_attr, 0x01, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &custom_value);
    esp_zb_custom_cluster_add_custom_attr(custom_attr, 0x02, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, custom_string);
    esp_zb_custom_cluster_add_custom_attr(custom_attr, 0x03, ESP_ZB_ZCL_ATTR_TYPE_32BIT_ARRAY, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, custom_array);

    char fw_version[32] = {0};
    fw_version[0] = 0;
    esp_zb_custom_cluster_add_custom_attr(custom_attr, LD2450_VERSION_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                                         ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, fw_version);

    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);


    //

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0,
    };

    /* Mandatory clusters */
    esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, HA_ESP_SENSOR_ENDPOINT, &info);

    esp_zb_device_register(ep_list);

    esp_zb_zcl_custom_cluster_handlers_t obj = {.cluster_id = CUSTOM_CLUSTER_ID,
                                                .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                .check_value_cb = esp_zb_zcl_cluster_check_value_handler,
                                                .write_attr_cb = esp_zb_zcl_cluster_write_attr_handler};
    esp_zb_zcl_custom_cluster_handlers_update(obj);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void) {
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    radar_data_mutex = xSemaphoreCreateMutex();
    if (radar_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create radar data mutex");
        return;
    }

    ld2450_uart_init();

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    vTaskDelay(pdMS_TO_TICKS(5000)); // wait until zigbee init
    ld2450_get_fw_version_and_update_zigbee();

    start_radar_processing();
}

