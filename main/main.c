#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

// Bibliotecas bluetooth
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_l2cap_bt_api.h"
#include "nvs_flash.h"
#include "esp_bt_defs.h"

// Bibliotecas FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_bd_addr_t remote_bda = {0}; // endereço do controle (preencher após o pareamento)
static bool device_found = false; // flag para indicar que o controle foi encontrado 

static bool device_initialized = false;
static int l2cap_step = 0;

static uint16_t handle_control = 0;
static uint16_t handle_interrupt = 0;
static int fd_control = -1;
static int fd_interrupt = -1;

typedef struct {
    uint8_t lx; //stick left x-axis
    uint8_t ly; //stick left y-axis
    uint8_t buttons1; //bt triangle, circle, cross, square, share, options, ps
    uint8_t buttons2; //bt l2, r2, l1, r1, left stick button, right stick button
    uint8_t l2; //left trigger (0-255)
    uint8_t r2; //right trigger (0-255)
} ds4_input_t;

void parse_ds4(uint8_t *d);
void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void l2cap_cb(esp_bt_l2cap_cb_event_t event, esp_bt_l2cap_cb_param_t *param);
void ds4_minimal_init(uint16_t cid_control);

// bt init OK
void bt_init(void) {
    // Inicializar NVS --- REVISADO ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configurar e iniciar o Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE)); // Liberar memória BLE, não precisamos
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)); // Importante: DS4 usa Bluetooth Clássico
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    esp_bt_gap_register_callback(gap_cb);
    esp_bt_l2cap_register_callback(l2cap_cb);

    ESP_ERROR_CHECK(esp_bt_l2cap_init());
    ESP_ERROR_CHECK(esp_bt_l2cap_vfs_register());

    // Configurar PIN fixo para pareamento (DS4 geralmente usa "1234")
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);

    // Configurar para não solicitar entrada de PIN (DS4 não tem interface de entrada)
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));
    
    // Configurar para encontrar DS4
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);

    printf("Bluetooth inicializado e scan iniciado...\n");
}

void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            char name[50] = {0};
            
            for (int i = 0; i < param->disc_res.num_prop; i++) {
                
                esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];

                switch (prop->type)
                {
                case ESP_BT_GAP_DEV_PROP_BDNAME:
                    memcpy(name, prop->val, prop->len);
                    printf("Dispositivo encontrado: %s\n", name);
                    name[prop->len] = '\0'; // Garantir terminação nula
                    break;
                
                case ESP_BT_GAP_DEV_PROP_EIR:
                    uint8_t len;
                    uint8_t *r = esp_bt_gap_resolve_eir_data((uint8_t *)prop->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);

                    if (r && len < sizeof(name)) {
                        memcpy(name, r, len);
                        name[len] = '\0'; // Garantir terminação nula
                        printf("Nome completo do dispositivo: %s\n", name);
                    }
                    break;

                default:
                    break;
                }
            }
            break;
        }

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                if (device_found) {
                    printf("Controle encontrado!\n");
                    
                    esp_bt_l2cap_connect(ESP_BT_L2CAP_SEC_AUTHENTICATE, 0x11, remote_bda);
                    esp_bt_l2cap_connect(ESP_BT_L2CAP_SEC_AUTHENTICATE, 0x13, remote_bda);
                } else {
                    printf("Nenhum controle encontrado. Reiniciando busca...\n");
                    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
                }
            }
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
        case ESP_BT_L2CAP_CL_INIT_EVT:
            // Este evento ocorre logo após o esp_bt_l2cap_connect
            // Use-o para salvar qual handle pertence a qual PSM
            if (l2cap_step == 0) {
                handle_control = param->cl_init.handle;
                printf("Canal de CONTROLE (0x11) conectado, handle: %lu\n", param->cl_init.handle);
            } else if (l2cap_step == 1) {
                handle_interrupt = param->cl_init.handle;
                printf("Canal de INTERRUPÇÃO (0x13) conectado, handle: %lu\n", param->cl_init.handle);
            }

            l2cap_step++;
            break;

        case ESP_BT_L2CAP_OPEN_EVT:
            if (param->open.status == ESP_BT_L2CAP_SUCCESS) {
                printf("L2CAP aberto! fd: %d, MTU: %d, Handle: %lu\n", 
                       param->open.fd, (int)param->open.tx_mtu, param->open.handle);

                if (param->open.handle == handle_control) {
                    fd_control = param->open.fd;
                    printf("Canal de CONTROLE (0x11) pronto!\n");
                } 
                else if (param->open.handle == handle_interrupt) {
                    fd_interrupt = param->open.fd;
                    printf("Canal de INTERRUPÇÃO (0x13) pronto!\n");
                }
            }
            break;

        case ESP_BT_L2CAP_CLOSE_EVT:
            printf("L2CAP fechado, Handle: 0x%lx, Status: %d\n", 
                   param->close.handle, param->close.status);

            l2cap_step = 0; // Resetar etapa para futuras conexões
            
            if (param->close.handle == handle_interrupt ||
                param->close.handle == handle_control) {
                device_found = false;
                device_initialized = false;
                handle_interrupt = -1;
                handle_control = -1;
                handle_control = 0;
                handle_interrupt = 0;
                
                printf("Conexão perdida. Reiniciando scan...\n");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            }
            break;

        default:
            break;
    }
}

// Esta função pode ser chamada após os canais estarem prontos para ler os dados do controle
void ds4_read_task(void *arg) {
    uint8_t buffer[128];

    while (1) {
        if (fd_interrupt > 0) { // Verificar se o canal de interrupção está aberto
            int len = read(fd_interrupt, buffer, sizeof(buffer)); // Ler dados do canal de interrupção (entrada do controle)

            if (len > 0) {
                if (buffer[0] != 0x01) { // O DS4 geralmente envia pacotes de 64 bytes, mas o primeiro byte pode ser um identificador. Verifique isso para evitar parsing incorreto.
                    printf("Relatório: 0x%02X\n", buffer[0]); // Debug: mostrar o primeiro byte do relatório para entender o formato dos dados
                    continue; // Ignorar pacotes que não sejam de entrada
                }
                parse_ds4(buffer);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Pequena pausa para evitar uso excessivo de CPU
    }
}

void parse_ds4(uint8_t *d)
{
    //stick esquerdo (x e y)
    uint8_t lx = d[1];
    uint8_t ly = d[2];

    //botões principais e ombros
    uint8_t b1 = d[5];
    uint8_t b2 = d[6];

    //gatilhos analógicos   
    uint8_t l2 = d[8];
    uint8_t r2 = d[9];

    //--- byte 5 ---
    // Botões principais
    bool square   = b1 & (1 << 4);
    bool cross    = b1 & (1 << 5);
    bool circle   = b1 & (1 << 6);
    bool triangle = b1 & (1 << 7);

    //--- byte 6 ---
    // Ombros
    bool l1 = b2 & (1 << 0);
    bool r1 = b2 & (1 << 1);

    // Stick pressionado
    bool l3 = b2 & (1 << 6);

    // Debug simples
    printf("LX:%3d LY:%3d | ", lx, ly);

    if (square)   printf("[ ] ");
    if (cross)    printf("X ");
    if (circle)   printf("O ");
    if (triangle) printf("/\\ ");

    if (l1) printf("L1 ");
    if (r1) printf("R1 ");

    if (l3) printf("L3 "); //click do stick esquerdo

    printf("L2:%3d R2:%3d", l2, r2); //"grau de pressão" dos gatilhos analógicos

    // Espaços para limpar linha antiga
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
