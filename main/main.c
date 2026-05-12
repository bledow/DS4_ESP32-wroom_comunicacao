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

static const char *TAG = "TESTE_HID_3";

static esp_bd_addr_t s_ds4_bda = {0};
static bool s_ds4_found = false;
static bool s_connect_started = false;

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

static void start_hid_connection(void)
{
    if (!s_ds4_found) {
        ESP_LOGW(TAG, "Nao ha controle salvo para conectar");
        return;
    }

    if (s_connect_started) {
        ESP_LOGW(TAG, "Conexao ja foi iniciada, ignorando nova tentativa");
        return;
    }

    s_connect_started = true;

    print_bda("Iniciando conexao HID com:", s_ds4_bda);

    esp_err_t ret = esp_bt_hid_host_connect(s_ds4_bda);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao chamar esp_bt_hid_host_connect: %s", esp_err_to_name(ret));
        s_connect_started = false;
    } else {
        ESP_LOGI(TAG, "esp_bt_hid_host_connect chamado com sucesso");
        ESP_LOGI(TAG, "Agora aguarde o evento ESP_HIDH_OPEN_EVT");
    }
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
                ESP_LOGI(TAG, "Agora vamos tentar conectar via HID Host");
                start_hid_connection();
            } else {
                ESP_LOGW(TAG, "Controle nao encontrado");
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
            "Dispositivo: %02X:%02X:%02X:%02X:%02X:%02X | nome='%s'",
            param->disc_res.bda[0], param->disc_res.bda[1],
            param->disc_res.bda[2], param->disc_res.bda[3],
            param->disc_res.bda[4], param->disc_res.bda[5],
            tem_nome ? nome : "<sem_nome>"
        );

        if (tem_rssi) {
            ESP_LOGI(TAG, "RSSI: %d", rssi);
        }

        if (tem_cod) {
            ESP_LOGI(TAG, "COD: 0x%06lX", (unsigned long) cod);
        }

        if (tem_nome && strcmp(nome, "Wireless Controller") == 0) {
            ESP_LOGI(TAG, "Controle DualShock 4 localizado pelo nome");

            memcpy(s_ds4_bda, param->disc_res.bda, sizeof(esp_bd_addr_t));
            s_ds4_found = true;

            print_bda("Endereco do controle:", s_ds4_bda);

            ESP_LOGI(TAG, "Cancelando scan antes de conectar");
            esp_bt_gap_cancel_discovery();
        }

        break;
    }

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Pareamento/autenticacao concluido com sucesso");
            ESP_LOGI(TAG, "Nome autenticado: %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Falha no pareamento/autenticacao. Status: %d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "Pedido de confirmacao SSP recebido");
        ESP_LOGI(TAG, "Valor numerico: %lu", (unsigned long) param->cfm_req.num_val);
        ESP_LOGI(TAG, "Confirmando automaticamente");
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(TAG, "Pedido de PIN recebido");

        esp_bt_pin_code_t pin_code;
        memset(pin_code, '0', ESP_BT_PIN_CODE_LEN);

        uint8_t pin_len = param->pin_req.min_16_digit ? 16 : 4;

        ESP_LOGI(TAG, "Respondendo PIN com %d zeros", pin_len);
        esp_bt_gap_pin_reply(param->pin_req.bda, true, pin_len, pin_code);
        break;
    }

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "Passkey notificado: %lu", (unsigned long) param->key_notif.passkey);
        break;

    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGW(TAG, "Controle pediu passkey. Neste teste nao vamos tratar entrada manual.");
        break;

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

    case ESP_HIDH_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_HIDH_OPEN_EVT recebido");
        ESP_LOGI(TAG, "Status open: %d", param->open.status);
        ESP_LOGI(TAG, "Conn status: %d", param->open.conn_status);
        ESP_LOGI(TAG, "Handle: %d", param->open.handle);
        ESP_LOGI(TAG, "Conexao iniciada pelo host? %s", param->open.is_orig ? "sim" : "nao");
        print_bda("Endereco conectado:", param->open.bd_addr);

        if (param->open.status == ESP_HIDH_OK &&
            param->open.conn_status == ESP_HIDH_CONN_STATE_CONNECTED) {

            ESP_LOGI(TAG, "SUCESSO: controle conectado via HID Host");
            ESP_LOGI(TAG, "Agora mexa nos analogicos e aperte botoes");
            ESP_LOGI(TAG, "Se chegarem reports, veremos ESP_HIDH_DATA_IND_EVT");
        } else {
            ESP_LOGE(TAG, "Falha ao conectar HID");
            ESP_LOGE(TAG, "Status=%d | ConnStatus=%d", param->open.status, param->open.conn_status);
        }

        break;

    case ESP_HIDH_DATA_IND_EVT:
        ESP_LOGI(TAG, "ESP_HIDH_DATA_IND_EVT recebido");
        ESP_LOGI(TAG, "Status: %d", param->data_ind.status);
        ESP_LOGI(TAG, "Handle: %d", param->data_ind.handle);
        ESP_LOGI(TAG, "Proto mode: %d", param->data_ind.proto_mode);
        ESP_LOGI(TAG, "Len: %d", param->data_ind.len);

        if (param->data_ind.len > 0 && param->data_ind.data != NULL) {
            ESP_LOGI(TAG, "Primeiro byte/report id: 0x%02X", param->data_ind.data[0]);
            ESP_LOG_BUFFER_HEX(TAG, param->data_ind.data, param->data_ind.len);
        }
        break;

    case ESP_HIDH_CLOSE_EVT:
        ESP_LOGW(TAG, "ESP_HIDH_CLOSE_EVT recebido");
        ESP_LOGW(TAG, "Status close: %d", param->close.status);
        ESP_LOGW(TAG, "Reason: %d", param->close.reason);
        ESP_LOGW(TAG, "Conn status: %d", param->close.conn_status);
        ESP_LOGW(TAG, "Handle: %d", param->close.handle);
        s_connect_started = false;
        break;

    case ESP_HIDH_GET_DSCP_EVT:
        ESP_LOGI(TAG, "ESP_HIDH_GET_DSCP_EVT recebido");
        ESP_LOGI(TAG, "Status descriptor: %d", param->dscp.status);
        ESP_LOGI(TAG, "Vendor ID: 0x%04X", param->dscp.vendor_id);
        ESP_LOGI(TAG, "Product ID: 0x%04X", param->dscp.product_id);
        ESP_LOGI(TAG, "Version: 0x%04X", param->dscp.version);
        ESP_LOGI(TAG, "Descriptor length: %d", param->dscp.dl_len);
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
    printf("ESP32 DualShock 4 Receiver - Teste HID 3\n");
    printf("Scan + Connect HID Host\n");
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

    /*
        Configuracao simples de seguranca.

        ESP_BT_IO_CAP_NONE significa que o ESP32 nao tem tela nem teclado
        para confirmar codigos manualmente.
    */
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(
        esp_bt_gap_set_security_param(
            ESP_BT_SP_IOCAP_MODE,
            &iocap,
            sizeof(uint8_t)
        )
    );

    ESP_LOGI(TAG, "Registrando callback GAP");
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));

    ESP_LOGI(TAG, "Registrando callback HID Host");
    ESP_ERROR_CHECK(esp_bt_hid_host_register_callback(hidh_callback));

    ESP_LOGI(TAG, "Inicializando HID Host");
    ESP_ERROR_CHECK(esp_bt_hid_host_init());

    ESP_LOGI(TAG, "app_main finalizado. Aguardando eventos...");
}