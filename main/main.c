#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hidh_api.h"

static const char *TAG = "TESTE_HID_2";

static esp_bd_addr_t s_ds4_bda = {0};
static bool s_ds4_found = false;

static void print_bda(const char *prefix, esp_bd_addr_t bda)
{
    ESP_LOGI(
        TAG,
        "%s %02X:%02X:%02X:%02X:%02X:%02X",
        prefix,
        bda[0], bda[1], bda[2],
        bda[3], bda[4], bda[5]
    );
}

static bool get_name_from_eir(uint8_t *eir, char *name, size_t name_size)
{
    if (eir == NULL || name == NULL || name_size == 0) {
        return false;
    }

    uint8_t name_len = 0;
    uint8_t *name_ptr = NULL;

    name_ptr = esp_bt_gap_resolve_eir_data(
        eir,
        ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME,
        &name_len
    );

    if (name_ptr == NULL) {
        name_ptr = esp_bt_gap_resolve_eir_data(
            eir,
            ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME,
            &name_len
        );
    }

    if (name_ptr == NULL || name_len == 0) {
        return false;
    }

    size_t copy_len = name_len;

    if (copy_len >= name_size) {
        copy_len = name_size - 1;
    }

    memcpy(name, name_ptr, copy_len);
    name[copy_len] = '\0';

    return true;
}

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(TAG, "Scan Bluetooth iniciado");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "Scan Bluetooth finalizado");

            if (s_ds4_found) {
                print_bda("Controle encontrado no endereco:", s_ds4_bda);
                ESP_LOGI(TAG, "Teste 2 passou: o ESP32 encontrou o controle");
            } else {
                ESP_LOGW(TAG, "Teste 2 ainda nao passou: controle nao encontrado");
                ESP_LOGW(TAG, "Coloque o DS4 em pareamento: SHARE + PS ate piscar");
            }
        }
        break;

    case ESP_BT_GAP_DISC_RES_EVT:
    {
        char nome[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {0};
        int8_t rssi = 0;
        uint32_t cod = 0;
        bool tem_nome = false;
        bool tem_rssi = false;
        bool tem_cod = false;

        for (int i = 0; i < param->disc_res.num_prop; i++) {

            esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];

            switch (prop->type) {

            case ESP_BT_GAP_DEV_PROP_BDNAME:
            {
                size_t copy_len = prop->len;

                if (copy_len >= sizeof(nome)) {
                    copy_len = sizeof(nome) - 1;
                }

                memcpy(nome, prop->val, copy_len);
                nome[copy_len] = '\0';
                tem_nome = true;
                break;
            }

            case ESP_BT_GAP_DEV_PROP_EIR:
                if (!tem_nome) {
                    tem_nome = get_name_from_eir(
                        (uint8_t *) prop->val,
                        nome,
                        sizeof(nome)
                    );
                }
                break;

            case ESP_BT_GAP_DEV_PROP_RSSI:
                rssi = *(int8_t *) prop->val;
                tem_rssi = true;
                break;

            case ESP_BT_GAP_DEV_PROP_COD:
                cod = *(uint32_t *) prop->val;
                tem_cod = true;
                break;

            default:
                break;
            }
        }

        ESP_LOGI(
            TAG,
            "Dispositivo encontrado: %02X:%02X:%02X:%02X:%02X:%02X | nome='%s'%s%d%s0x%06lX",
            param->disc_res.bda[0], param->disc_res.bda[1],
            param->disc_res.bda[2], param->disc_res.bda[3],
            param->disc_res.bda[4], param->disc_res.bda[5],
            tem_nome ? nome : "<sem_nome>",
            tem_rssi ? " | RSSI=" : "",
            tem_rssi ? rssi : 0,
            tem_cod ? " | COD=" : "",
            tem_cod ? cod : 0
        );

        if (tem_nome && strcmp(nome, "Wireless Controller") == 0) {
            ESP_LOGI(TAG, "Controle DualShock 4 localizado pelo nome");

            memcpy(s_ds4_bda, param->disc_res.bda, sizeof(esp_bd_addr_t));
            s_ds4_found = true;

            print_bda("Endereco do controle:", s_ds4_bda);

            ESP_LOGI(TAG, "Cancelando scan. Neste teste ainda nao vamos conectar.");
            esp_bt_gap_cancel_discovery();
        }

        break;
    }

    default:
        ESP_LOGI(TAG, "Evento GAP recebido: %d", event);
        break;
    }
}

static void hidh_callback(esp_hidh_cb_event_t event, esp_hidh_cb_param_t *param)
{
    switch (event) {

    case ESP_HIDH_INIT_EVT:
        ESP_LOGI(TAG, "ESP_HIDH_INIT_EVT recebido");
        ESP_LOGI(TAG, "Status HID Host: %d", param->init.status);

        if (param->init.status == ESP_HIDH_OK) {
            ESP_LOGI(TAG, "HID Host OK");
            ESP_LOGI(TAG, "Iniciando scan Bluetooth Classic...");

            esp_err_t ret = esp_bt_gap_start_discovery(
                ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                10,
                0
            );

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Erro ao iniciar scan: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Erro ao inicializar HID Host");
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

    printf("\n");
    printf("========================================\n");
    printf("ESP32 DualShock 4 Receiver - Teste HID 2\n");
    printf("Scan Bluetooth Classic\n");
    printf("========================================\n\n");

    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_LOGW(TAG, "NVS precisa ser apagado. Apagando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "NVS Flash inicializado");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_LOGI(TAG, "Inicializando Bluetooth Controller");
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_LOGI(TAG, "Habilitando Bluetooth Controller em modo BTDM");
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));

    ESP_LOGI(TAG, "Inicializando Bluedroid");

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
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

    ESP_LOGI(TAG, "Registrando callback GAP");
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));

    ESP_LOGI(TAG, "Registrando callback HID Host");
    ESP_ERROR_CHECK(esp_bt_hid_host_register_callback(hidh_callback));

    ESP_LOGI(TAG, "Inicializando HID Host");
    ESP_ERROR_CHECK(esp_bt_hid_host_init());

    ESP_LOGI(TAG, "app_main finalizado. Aguardando eventos...");
}