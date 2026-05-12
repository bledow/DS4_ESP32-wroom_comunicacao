#include <stdio.h>
#include <string.h>

// Bibliotecas bluetooth
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_hidh_api.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_bt_device.h"

// Bibliotecas FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "TESTE_HID_1";

static void hidh_callback(esp_hidh_cb_event_t event, esp_hidh_cb_param_t *param)
{
    switch (event) {
        case ESP_HIDH_INIT_EVT:
            ESP_LOGI(TAG, "ESP_HIDH_INIT_EVT recebido");
            ESP_LOGI(TAG, "Status da inicializacao HID Host: %d", param->init.status);
            if(param->init.status == ESP_HIDH_OK) {
                ESP_LOGI(TAG, "SUCESSO: HID Host inicializado corretamente");
            } else {
                ESP_LOGE(TAG, "ERRO: HID Host nao inicializou corretamente");
            }
            break;

        default:
            ESP_LOGI(TAG, "Evento HID recebido: %d", event);
            break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    printf("\n========================================\n");
    printf("ESP32 DualShock 4 Receiver - Teste HID 1\n");
    printf("Bluetooth classic + HID Host\n");
    printf("========================================\n\n");

    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "NVS Flash inicializado... \n");

    //ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_LOGI(TAG, "Inicializando Bluetooth Controller");
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_LOGI(TAG, "Habilitando Bluetooth Controller em modo Classic");
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    ESP_LOGI(TAG, "Inicializando Bluedroid");
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));

    ESP_LOGI(TAG, "Habilitando Bluedroid");
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    const uint8_t *bt_addr = esp_bt_dev_get_address();

    ESP_LOGI(
        TAG,
        "Endereco Bluetooth do ESP32: %02X:%02X:%02X:%02X:%02X:%02X",
        bt_addr[0], bt_addr[1], bt_addr[2],
        bt_addr[3], bt_addr[4], bt_addr[5]
    );

    ESP_LOGI(TAG, "Registrando callback HID Host");
    ESP_ERROR_CHECK(esp_bt_hid_host_register_callback(hidh_callback));

    ESP_LOGI(TAG, "Inicializando HID Host");
    ESP_ERROR_CHECK(esp_bt_hid_host_init());

    ESP_LOGI(TAG, "app_main finalizado. Agora aguarde o callback ESP_HIDH_INIT_EVT.");
    
}