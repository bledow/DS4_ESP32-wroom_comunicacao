#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <inttypes.h>

// Bibliotecas bluetooth
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_l2cap_bt_api.h"
#include "nvs_flash.h"
#include "esp_bt_defs.h"
#include "esp_err.h"

// Bibliotecas FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DS4_NAME              "Wireless Controller"
#define DS4_PSM_CONTROL       0x11
#define DS4_PSM_INTERRUPT     0x13
#define DS4_SEC_MASK          ESP_BT_L2CAP_SEC_AUTHENTICATE

static esp_bd_addr_t remote_bda = {0};
static bool device_found = false;
static bool discovery_started = false;
static bool read_task_started = false;

static uint32_t handle_control = 0;
static uint32_t handle_interrupt = 0;
static int fd_control = -1;
static int fd_interrupt = -1;

typedef enum {
    L2CAP_STATE_IDLE = 0,
    L2CAP_STATE_CONNECTING_CONTROL,
    L2CAP_STATE_CONNECTING_INTERRUPT,
    L2CAP_STATE_READY
} l2cap_state_t;

static l2cap_state_t l2cap_state = L2CAP_STATE_IDLE;

typedef struct {
    uint8_t lx;       // stick left x-axis
    uint8_t ly;       // stick left y-axis
    uint8_t buttons1; // triangle, circle, cross, square, share, options, ps
    uint8_t buttons2; // l2, r2, l1, r1, left stick button, right stick button
    uint8_t l2;       // left trigger (0-255)
    uint8_t r2;       // right trigger (0-255)
} ds4_input_t;

void parse_ds4(uint8_t *d);
void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void l2cap_cb(esp_bt_l2cap_cb_event_t event, esp_bt_l2cap_cb_param_t *param);
void ds4_read_task(void *arg);

static void start_scan(void);
static void connect_control_channel(void);
static void connect_interrupt_channel(void);
static bool copy_eir_name(uint8_t *eir, char *name, size_t name_size);
static void reset_connection_state(void);

static void start_scan(void)
{
    if (discovery_started) {
        return;
    }

    device_found = false;
    memset(remote_bda, 0, sizeof(remote_bda));

    esp_err_t ret = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    if (ret == ESP_OK) {
        discovery_started = true;
        printf("Bluetooth inicializado e scan iniciado...\n");
    } else {
        printf("Falha ao iniciar discovery: %s\n", esp_err_to_name(ret));
    }
}

static void connect_control_channel(void)
{
    if (l2cap_state != L2CAP_STATE_IDLE) {
        return;
    }

    l2cap_state = L2CAP_STATE_CONNECTING_CONTROL;
    printf("Abrindo canal L2CAP de CONTROLE PSM 0x%02X...\n", DS4_PSM_CONTROL);

    esp_err_t ret = esp_bt_l2cap_connect(DS4_SEC_MASK, DS4_PSM_CONTROL, remote_bda);
    if (ret != ESP_OK) {
        printf("Falha ao iniciar conexão L2CAP controle: %s\n", esp_err_to_name(ret));
        reset_connection_state();
        start_scan();
    }
}

static void connect_interrupt_channel(void)
{
    if (l2cap_state != L2CAP_STATE_CONNECTING_CONTROL || fd_control < 0) {
        return;
    }

    l2cap_state = L2CAP_STATE_CONNECTING_INTERRUPT;
    printf("Abrindo canal L2CAP de INTERRUPÇÃO PSM 0x%02X...\n", DS4_PSM_INTERRUPT);

    esp_err_t ret = esp_bt_l2cap_connect(DS4_SEC_MASK, DS4_PSM_INTERRUPT, remote_bda);
    if (ret != ESP_OK) {
        printf("Falha ao iniciar conexão L2CAP interrupção: %s\n", esp_err_to_name(ret));
        reset_connection_state();
        start_scan();
    }
}

static bool copy_eir_name(uint8_t *eir, char *name, size_t name_size)
{
    uint8_t len = 0;
    uint8_t *r = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);

    if (!r) {
        r = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
    }

    if (r && len > 0 && name_size > 0) {
        size_t copy_len = len < (name_size - 1) ? len : (name_size - 1);
        memcpy(name, r, copy_len);
        name[copy_len] = '\0';
        return true;
    }

    return false;
}

static void reset_connection_state(void)
{
    l2cap_state = L2CAP_STATE_IDLE;
    handle_control = 0;
    handle_interrupt = 0;
    fd_control = -1;
    fd_interrupt = -1;
    device_found = false;
    read_task_started = false;
}

void bt_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_bt_l2cap_register_callback(l2cap_cb));

    esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code));

    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap)));

    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    // NÃO chame esp_bt_l2cap_vfs_register() aqui.
    // esp_bt_l2cap_init() é assíncrono; o VFS deve ser registrado no ESP_BT_L2CAP_INIT_EVT.
    ESP_ERROR_CHECK(esp_bt_l2cap_init());

    printf("Bluetooth Classic habilitado. Aguardando ESP_BT_L2CAP_INIT_EVT...\n");
}

void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {0};
            bool has_name = false;

            for (int i = 0; i < param->disc_res.num_prop; i++) {
                esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];

                switch (prop->type) {
                    case ESP_BT_GAP_DEV_PROP_BDNAME: {
                        size_t copy_len = prop->len < ESP_BT_GAP_MAX_BDNAME_LEN ? prop->len : ESP_BT_GAP_MAX_BDNAME_LEN;
                        memcpy(name, prop->val, copy_len);
                        name[copy_len] = '\0';
                        has_name = true;
                        break;
                    }

                    case ESP_BT_GAP_DEV_PROP_EIR:
                        if (!has_name) {
                            has_name = copy_eir_name((uint8_t *)prop->val, name, sizeof(name));
                        }
                        break;

                    default:
                        break;
                }
            }

            if (has_name) {
                printf("Dispositivo encontrado: %s\n", name);
            }

            if (has_name && strcmp(name, DS4_NAME) == 0 && !device_found) {
                device_found = true;
                memcpy(remote_bda, param->disc_res.bda, sizeof(remote_bda));

                printf("Controle PS4 encontrado. Cancelando discovery para conectar...\n");
                esp_err_t ret = esp_bt_gap_cancel_discovery();
                if (ret != ESP_OK) {
                    printf("Falha ao cancelar discovery: %s\n", esp_err_to_name(ret));
                }
            }
            break;
        }

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                discovery_started = true;
                printf("Discovery iniciado. Coloque o DS4 em modo pareamento.\n");
            } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                discovery_started = false;

                if (device_found) {
                    printf("Discovery parado. Iniciando conexão com o controle...\n");
                    connect_control_channel();
                } else {
                    printf("Nenhum controle encontrado. Reiniciando busca...\n");
                    start_scan();
                }
            }
            break;
        }

        case ESP_BT_GAP_PIN_REQ_EVT: {
            esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
            printf("PIN solicitado. Respondendo 1234...\n");
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;
        }

        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                printf("Pareamento OK com %s\n", param->auth_cmpl.device_name);
            } else {
                printf("Falha no pareamento: %d\n", param->auth_cmpl.stat);
            }
            break;
        }

        default:
            break;
    }
}

void l2cap_cb(esp_bt_l2cap_cb_event_t event, esp_bt_l2cap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_L2CAP_INIT_EVT:
            printf("ESP_BT_L2CAP_INIT_EVT: status=%d\n", param->init.status);
            if (param->init.status == ESP_BT_L2CAP_SUCCESS) {
                esp_err_t ret = esp_bt_l2cap_vfs_register();
                if (ret != ESP_OK) {
                    printf("Falha ao registrar VFS L2CAP: %s\n", esp_err_to_name(ret));
                }
            }
            break;

        case ESP_BT_L2CAP_VFS_REGISTER_EVT:
            printf("ESP_BT_L2CAP_VFS_REGISTER_EVT: status=%d\n", param->vfs_register.status);
            if (param->vfs_register.status == ESP_BT_L2CAP_SUCCESS) {
                start_scan();
            }
            break;

        case ESP_BT_L2CAP_CL_INIT_EVT:
            printf("ESP_BT_L2CAP_CL_INIT_EVT: status=%d, handle=%" PRIu32 "\n",
                   param->cl_init.status, param->cl_init.handle);

            if (param->cl_init.status != ESP_BT_L2CAP_SUCCESS) {
                printf("Falha ao iniciar canal L2CAP. Voltando ao scan.\n");
                reset_connection_state();
                start_scan();
                break;
            }

            if (l2cap_state == L2CAP_STATE_CONNECTING_CONTROL) {
                handle_control = param->cl_init.handle;
            } else if (l2cap_state == L2CAP_STATE_CONNECTING_INTERRUPT) {
                handle_interrupt = param->cl_init.handle;
            }
            break;

        case ESP_BT_L2CAP_OPEN_EVT:
            printf("ESP_BT_L2CAP_OPEN_EVT: status=%d, fd=%d, MTU=%" PRId32 ", handle=%" PRIu32 "\n",
                   param->open.status, param->open.fd, param->open.tx_mtu, param->open.handle);

            if (param->open.status != ESP_BT_L2CAP_SUCCESS) {
                printf("Falha ao abrir canal L2CAP. Voltando ao scan.\n");
                reset_connection_state();
                start_scan();
                break;
            }

            if (l2cap_state == L2CAP_STATE_CONNECTING_CONTROL && param->open.handle == handle_control) {
                fd_control = param->open.fd;
                printf("Canal de CONTROLE pronto.\n");
                connect_interrupt_channel();
            } else if (l2cap_state == L2CAP_STATE_CONNECTING_INTERRUPT && param->open.handle == handle_interrupt) {
                fd_interrupt = param->open.fd;
                l2cap_state = L2CAP_STATE_READY;
                printf("Canal de INTERRUPÇÃO pronto. Controle inicializado.\n");

                if (!read_task_started) {
                    read_task_started = true;
                    xTaskCreate(ds4_read_task, "ds4_read_task", 4096, NULL, 5, NULL);
                }
            }
            break;

        case ESP_BT_L2CAP_CLOSE_EVT:
            printf("ESP_BT_L2CAP_CLOSE_EVT: handle=%" PRIu32 ", status=%d\n",
                   param->close.handle, param->close.status);

            if (param->close.handle == handle_interrupt || param->close.handle == handle_control) {
                printf("Conexão perdida. Reiniciando scan...\n");
                reset_connection_state();
                start_scan();
            }
            break;

        default:
            break;
    }
}

void ds4_read_task(void *arg)
{
    uint8_t buffer[128];

    while (1) {
        if (fd_interrupt >= 0) {
            int len = read(fd_interrupt, buffer, sizeof(buffer));

            if (len > 0) {
                if (buffer[0] != 0x01) {
                    printf("Relatório: 0x%02X\n", buffer[0]);
                    continue;
                }
                parse_ds4(buffer);
            } else if (len < 0) {
                printf("Erro ao ler canal de interrupção. Encerrando task.\n");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    read_task_started = false;
    vTaskDelete(NULL);
}

void parse_ds4(uint8_t *d)
{
    uint8_t lx = d[1];
    uint8_t ly = d[2];

    uint8_t b1 = d[5];
    uint8_t b2 = d[6];

    uint8_t l2 = d[8];
    uint8_t r2 = d[9];

    bool square   = b1 & (1 << 4);
    bool cross    = b1 & (1 << 5);
    bool circle   = b1 & (1 << 6);
    bool triangle = b1 & (1 << 7);

    bool l1 = b2 & (1 << 0);
    bool r1 = b2 & (1 << 1);
    bool l3 = b2 & (1 << 6);

    printf("LX:%3d LY:%3d | ", lx, ly);

    if (square)   printf("[ ] ");
    if (cross)    printf("X ");
    if (circle)   printf("O ");
    if (triangle) printf("/\\ ");

    if (l1) printf("L1 ");
    if (r1) printf("R1 ");
    if (l3) printf("L3 ");

    printf("L2:%3d R2:%3d", l2, r2);
    printf("                \r");
    fflush(stdout);
}

void app_main(void)
{
    printf("\n========================================\n");
    printf("ESP32 DualShock 4 Receiver - Modo Básico\n");
    printf("Aguardando conexão do controle...\n");
    printf("========================================\n\n");

    bt_init();
}
