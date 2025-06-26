#include <stdio.h> // incluir biblioteca de entrada e saída padrão
#include <string.h> // incluir biblioteca de manipulação de strings
#include "driver/i2c.h" // usar barra de i2c para leitura de sensores
#include "driver/gpio.h" // usar barra de gpio para controle de pinos
#include "esp_log.h" // usar barra de log para mensagens de depuração
#include "freertos/FreeRTOS.h" // usar barra de FreeRTOS para tarefas e filas
#include "freertos/task.h" // usar barra de tarefa para criar tarefas
#include "freertos/queue.h" // usar barra de fila para comunicação entre tarefas
#include "freertos/event_groups.h" // usar barra de eventos para sinalização de eventos
#include "esp_system.h" // usar barra de sistema para funções do sistema
#include <math.h> // usar barra de matemática para cálculos matemáticos
#include "driver/uart.h" // usar barra de uart para comunicação serial
#include "minmea.h" // parser de GPS NMEA
#include "mqtt_client.h" // usar barra de cliente MQTT para comunicação MQTT
#include "nvs_flash.h" // usar barra de NVS para armazenamento não volátil
#include "spi_flash_mmap.h" // usar barra de mapeamento de memória flash SPI para acessar a memória flash
#include "esp_wifi.h" // usar barra de Wi-Fi para conectar-se a uma rede Wi-Fi
#include "esp_event.h" 
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include <cJSON.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>
#include "lwip/dns.h"

// I2C pins
#define I2C_SDA GPIO_NUM_21  // I2C SDA pin
#define I2C_SCL GPIO_NUM_22  // I2C SCL pin

// I2C device addresses
#define MPU6050_ADDR  0x68
#define MPU6050_ACCEL_XOUT_H 0x3B

// Movement detection parameters
#define DELTA_THRESHOLD 0.2f //  Threshold for aceleration change 
#define MONITORAMENTO_DURACAO_MS 10000 // time of monitoring duration [ms]
#define CHECK_INTERVAL_MS 100 // Interval between checks [ms]
#define TOTAL_LEITURAS (MONITORAMENTO_DURACAO_MS / CHECK_INTERVAL_MS) // Total number of readings during monitoring
#define FRACAO_MINIMA_MOVIMENTO 0.6f  // porcentual mínimo de leituras que devem indicar movimento para considerar que houve movimento 

// GPS UART configuration
#define GPS_UART_PORT      UART_NUM_2
#define GPS_TXD_PIN        (GPIO_NUM_17) 
#define GPS_RXD_PIN        (GPIO_NUM_16) 
#define GPS_BAUD_RATE      9600

// GSM UART configuration
#define GSM_TXD            GPIO_NUM_10
#define GSM_RXD            GPIO_NUM_9
#define GSM_UART           UART_NUM_1
#define GSM_BAUD           9600

// Button GPIOs configuration
#define BTN_EMERGENCIA GPIO_NUM_18
#define BTN_ANTIFURTO  GPIO_NUM_19

// Reed switch GPIO configuration
#define REED_SWITCH    GPIO_NUM_23

// Buzzer GPIO configuration
#define BUZZER         GPIO_NUM_25

#define EXAMPLE_ESP_WIFI_SSID     "bobs"
#define EXAMPLE_ESP_WIFI_PASS     "abacaxiamarelo"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

#define POSITION_TOPIC "MochilaAntifurto/Position"

// Structure to hold IMU readings
typedef struct {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

// State machine for movement detection
typedef enum {
    EM_REPOUSO,
    MONITORANDO_MOVIMENTO
} estado_movimento_t;

// State machine for global operation
typedef enum {
    AWAIT,
    EMERGENCY_MODE,
    ANTI_THEFT_MODE,
    ALARMING
} estado_global_t;

mpu6050_data_t imu_data = {0}; // Global variable to hold IMU data
static const char *TAG = "MochilaAntiFurto"; 
float acc_offset_x = 0, acc_offset_y = 0, acc_offset_z = 0; // Calibration offsets for accelerometer
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0; // Calibration offsets for gyroscope
estado_global_t estado_global = AWAIT;
char gps_position[100] = "Position inconnue";
bool volatile mouvement_detecte = false; // Flag to indicate if movement is detected
float last_latitude = 0.0f, last_longitude = 0.0f; // GPS coordinates
int16_t last_longitude_int = 0, last_latitude_int = 0; // GPS coordinates as integers for NVS storage

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");
extern const uint8_t server_cert_pem_start[] asm("_binary_AmazonRootCA1_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_AmazonRootCA1_pem_end");

esp_mqtt_client_handle_t client_mqtt;

// Protótipos
static void IRAM_ATTR btn_emerg_isr_handler(void *arg);
static void IRAM_ATTR btn_antifurto_isr_handler(void *arg);
void buzzer_on();
void buzzer_off();
void publish_message_positon(const char *mensagem);
void save_calibration(float latitude, float longitude);

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void connect_wifi(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    vEventGroupDelete(s_wifi_event_group);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(client, "MochilaAntifurto", 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED: // only for QoS 1 and 2
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);

            int lvalue = 0;

            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

// Initialize I2C master for sensors
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

// Initialize MPU6050 IMU (wake up from sleep)
void mpu6050_init() {
    uint8_t wake_cmd[] = {0x6B, 0x00};
    i2c_master_write_to_device(I2C_NUM_0, MPU6050_ADDR, wake_cmd, 2, 1000 / portTICK_PERIOD_MS);
}

// Calibrate MPU6050 IMU by averaging multiple samples
void mpu6050_calibrate(int samples) {
    int16_t ax, ay, az, gx, gy, gz;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t data[14];
        i2c_master_write_read_device(I2C_NUM_0, MPU6050_ADDR, (uint8_t[]){MPU6050_ACCEL_XOUT_H}, 1, data, 14, 1000 / portTICK_PERIOD_MS);
        ax = (int16_t)(data[0] << 8 | data[1]);
        ay = (int16_t)(data[2] << 8 | data[3]);
        az = (int16_t)(data[4] << 8 | data[5]);
        gx = (int16_t)(data[8] << 8 | data[9]);
        gy = (int16_t)(data[10] << 8 | data[11]);
        gz = (int16_t)(data[12] << 8 | data[13]);

        sum_ax += ax;
        sum_ay += ay;
        sum_az += az; //- 16384; // Subtract 1g on Z
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    acc_offset_x = sum_ax / (float)samples / 16384.0f;
    acc_offset_y = sum_ay / (float)samples / 16384.0f;
    acc_offset_z = sum_az / (float)samples / 16384.0f;
    gyro_offset_x = sum_gx / (float)samples / 131.0f;
    gyro_offset_y = sum_gy / (float)samples / 131.0f;
    gyro_offset_z = sum_gz / (float)samples / 131.0f;

    ESP_LOGI(TAG, "MPU6050 calibrated.");
}

// Read data from MPU6050 IMU and apply calibration offsets
mpu6050_data_t mpu6050_read() {
    uint8_t data[14];
    mpu6050_data_t sensor_data = {0};
    esp_err_t ret = i2c_master_write_read_device(I2C_NUM_0, MPU6050_ADDR, (uint8_t[]){MPU6050_ACCEL_XOUT_H}, 1, data, 14, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 read failed: %s", esp_err_to_name(ret));
        return sensor_data;
    }

    sensor_data.acc_x = ((int16_t)(data[0] << 8 | data[1]) / 16384.0) - acc_offset_x;
    sensor_data.acc_y = ((int16_t)(data[2] << 8 | data[3]) / 16384.0) - acc_offset_y;
    sensor_data.acc_z = ((int16_t)(data[4] << 8 | data[5]) / 16384.0) - acc_offset_z;
    sensor_data.gyro_x = ((int16_t)(data[8] << 8 | data[9]) / 131.0) - gyro_offset_x;
    sensor_data.gyro_y = ((int16_t)(data[10] << 8 | data[11]) / 131.0) - gyro_offset_y;
    sensor_data.gyro_z = ((int16_t)(data[12] << 8 | data[13]) / 131.0) - gyro_offset_z;

    return sensor_data;
}

// Task to periodically read IMU data
void acelerometer_task(void *arg) {
    float last_x = 0, last_y = 0, last_z = 0;
    bool first_reading = true;
    float delta = 0.0f;
    estado_movimento_t estado = EM_REPOUSO;

    int contador_leituras = 0;
    int contador_movimento = 0;
    while (1) {
        if (estado_global == ANTI_THEFT_MODE) {
            imu_data = mpu6050_read();

            float acc_x = imu_data.acc_x;
            float acc_y = imu_data.acc_y;
            float acc_z = imu_data.acc_z;

            if (!first_reading) {
                delta = sqrtf(
                    powf(acc_x - last_x, 2) +
                    powf(acc_y - last_y, 2) +
                    powf(acc_z - last_z, 2)
                );
            }

            first_reading = false;

            switch (estado) {
                case EM_REPOUSO:
                    if (delta > DELTA_THRESHOLD) {
                        estado = MONITORANDO_MOVIMENTO;
                        contador_leituras = 1;
                        contador_movimento = 1;
                        ESP_LOGI(TAG, "Início de possível movimento (Δ=%.4f)", delta);
                    }
                    break;

                case MONITORANDO_MOVIMENTO:
                    contador_leituras++;
                    if (delta > DELTA_THRESHOLD) {
                        contador_movimento++;
                    }
                    if (contador_leituras >= TOTAL_LEITURAS) {
                        float proporcao = (float)contador_movimento / contador_leituras;
                        if (proporcao >= FRACAO_MINIMA_MOVIMENTO) {
                            ESP_LOGW(TAG, "Movimento suspeito detectado! Δ (%d/%d)", contador_movimento, contador_leituras);
                            mouvement_detecte = true;
                        } else {
                            ESP_LOGI(TAG, "Movimento descartado. Δ (%d/%d)", contador_movimento, contador_leituras);
                            mouvement_detecte = false;
                        }
                        estado = EM_REPOUSO;
                    }
                    break;
                }
        
            last_x = acc_x;
            last_y = acc_y;
            last_z = acc_z;
        }
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
    }
}

void gps_init() {
    const uart_config_t gps_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(GPS_UART_PORT, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_PORT, &gps_config);
    uart_set_pin(GPS_UART_PORT, GPS_TXD_PIN, GPS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Task to read GPS data and parse sentences
void gps_task(void *arg) {
    uint8_t data[128];
    char line[128];
    int pos = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT, data, sizeof(data) - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];
                // Trata linhas completas
                if (c == '\n' || c == '\r') {
                    if (pos == 0) continue;  // ignora quebras duplicadas

                    line[pos] = '\0';  // finaliza a string
                    //ESP_LOGI(TAG, "NMEA: %s", line);

                    // Verifica se é uma sentença GGA
                    if (minmea_sentence_id(line, false) == MINMEA_SENTENCE_GGA) {
                        struct minmea_sentence_gga frame;
                        if (minmea_parse_gga(&frame, line)) {
                            if (frame.fix_quality > 0) {
                                float lat = minmea_tofloat(&frame.latitude);
                                float lon = minmea_tofloat(&frame.longitude);
                                last_latitude = lat;
                                last_longitude = lon;
                                save_calibration(last_latitude, last_longitude);
                                ESP_LOGI(TAG, "Localização: Lat: %.5f, Lon: %.5f", lat, lon);
                            } else {
                                ESP_LOGW(TAG, "Sem fix GPS (fix_quality = %d)", frame.fix_quality);
                            }
                        } else {
                            ESP_LOGW(TAG, "Falha ao parsear GGA");
                        }
                    }
                    pos = 0;  // reinicia leitura da próxima sentença
                }
                // Armazena caracteres válidos
                else if (pos < sizeof(line) - 1 && (c >= 32 && c <= 126)) {
                    line[pos++] = c;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // pequena pausa para cooperatividade
    }
}

void buzzer_on() {
    gpio_set_level(BUZZER, 1);
}

void buzzer_off() {
    gpio_set_level(BUZZER, 0);
}

// detecta pressionamento dos botões por interrupção 
void botoes_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,  // PAS d'interruption
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BTN_EMERGENCIA) | (1ULL << BTN_ANTIFURTO) | (1ULL << REED_SWITCH),
        .pull_up_en = GPIO_PULLUP_ENABLE,  // bouton connecté à GND + pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
}

void botoes_task(void *arg) {
    ESP_LOGI(TAG, "botoes task");
    int last_emerg = 1;
    int last_antifurto = 1;

    while (1) {
        int estado_emerg = gpio_get_level(BTN_EMERGENCIA);
        int estado_antifurto = gpio_get_level(BTN_ANTIFURTO);

        // Gestion bouton urgence
        if (estado_emerg == 0) {
            if (estado_global == AWAIT) {
                ESP_LOGI(TAG, "Bouton urgence pressé, activation du mode urgence.");
                estado_global = EMERGENCY_MODE;
            } 
                
        } else if (estado_emerg == 1) {
            if (estado_global == EMERGENCY_MODE) {
                ESP_LOGI(TAG, "Bouton urgence relâché, désactivation du mode urgence.");
                estado_global = AWAIT;
            }
        }

        // Gestion bouton antifurto
        if (estado_antifurto == 0) {
            if (estado_global == AWAIT) {
                ESP_LOGI(TAG, "Bouton antifurto pressé, activation du mode anti-furto.");
                estado_global = ANTI_THEFT_MODE;
            }
        } else if (estado_antifurto ==1) {
            if (estado_global == ANTI_THEFT_MODE || estado_global == ALARMING) {
                ESP_LOGI(TAG, "Bouton antifurto pressé, desactivation du mode anti-furto.");
                estado_global = AWAIT;
            }}

        vTaskDelay(pdMS_TO_TICKS(100)); // Antirebond + polling
    }
}


void antifurto_task(void *arg) {
    while (1) {
        if (estado_global == ANTI_THEFT_MODE) {
            // Vérification ouverture (reed switch)
            if (gpio_get_level(REED_SWITCH) == 1) {
                ESP_LOGW(TAG, "Alerte : ouverture détectée !");
                buzzer_on();
                publish_message_positon("S0S - Abertura da Mochila detectada");
                estado_global = ALARMING;
                continue;
            }
            // Vérification du mouvement détecté par acelerometer_task
            if (mouvement_detecte) {
                ESP_LOGW(TAG, "Alerte : mouvement suspect détecté !");
                buzzer_on();
                publish_message_positon("S0S - Movimento suspeito detectado na Mochila");
                estado_global = ALARMING;
                mouvement_detecte = false; // Reset après détection
                continue;
            }
        } else if (estado_global == AWAIT) {
            buzzer_off();
            mouvement_detecte = false; // Remettre à zéro si on repasse à AWAIT
        }
        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
    }
}

void emergencia_task(void *arg) {
    TickType_t start_time = 0;
    while (1) {
        if (estado_global == EMERGENCY_MODE) {
            start_time = xTaskGetTickCount();
            publish_message_positon("S0S - Estado de emergencia ativado");
            ESP_LOGI(TAG, "SMS urgence envoyé avec position GPS.");
            // Attendre 3 minutes (180000 ms)
            vTaskDelay(pdMS_TO_TICKS(9000));
            buzzer_on();
            ESP_LOGI(TAG, "Buzzer activé en mode urgence.");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void mqtt_app_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtts://a2u5iulk6chw2o-ats.iot.us-east-1.amazonaws.com:8883",
        .broker.verification.certificate = (const char *)server_cert_pem_start,
        .credentials = {
            .authentication = {
                .certificate = (const char *)client_cert_pem_start,
                .key = (const char *)client_key_pem_start,
            },
        }
    };

    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    client_mqtt = client;
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "after mqtt client start");
}

void publish_message_positon(const char *mensagem) {
    if (client_mqtt == NULL) return;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "ID", "MochilaAntifurto - PMR");
    cJSON_AddNumberToObject(root, "LATITUDE", last_latitude);
    cJSON_AddNumberToObject(root, "LONGITUDE", last_longitude);
    cJSON_AddStringToObject(root, "MENSAGEM", mensagem);

    char *json_str = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client_mqtt, POSITION_TOPIC, json_str, strlen(json_str), 1, 0);
    cJSON_Delete(root);
    free(json_str);
}

void save_calibration(float latitude, float longitude) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("position_data", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Erro ao abrir NVS: %s", esp_err_to_name(err));
        return;
    }

    nvs_set_i16(handle, "last_latitude", latitude*10000);
    nvs_set_i16(handle, "last_longitude", longitude*10000);
    nvs_set_u8(handle, "saved", 1);  // marcar como calibrado

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Erro ao salvar ultimas coordenadas: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("NVS", "Ultima localização salva com sucesso.");
    }

    nvs_close(handle);
}

bool load_calibration() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("position_data", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW("NVS", "Erro ao abrir NVS: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t calibrated = 0;
    nvs_get_u8(handle, "saved", &calibrated);
    if (calibrated != 1) {
        ESP_LOGW("NVS", "Nenhuma calibração salva.");
        nvs_close(handle);
        return false;
    }

    nvs_get_i16(handle, "last_latitude", &last_latitude_int);
    nvs_get_i16(handle, "last_longitude", &last_longitude_int);
    last_latitude = last_latitude_int / 10000.0f; // Convertendo para float
    last_longitude = last_longitude_int/ 10000.0f; // Convertendo para float
    
    nvs_close(handle);
    return true;
}


void app_main() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    load_calibration();
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    connect_wifi();
    // Initialize MQTT
    mqtt_app_start();

    i2c_master_init(); 
    ESP_LOGI(TAG, "i2c Master initialized");
    mpu6050_init();
    mpu6050_calibrate(100);
    gps_init();
    botoes_init();

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(botoes_task, "botoes_task", 2048, NULL, 5, NULL);
    xTaskCreate(antifurto_task, "antifurto_task", 4096, NULL, 6, NULL);
    xTaskCreate(emergencia_task, "emergencia_task", 4096, NULL, 7, NULL);
    xTaskCreate(acelerometer_task, "acelerometer_task", 4096, NULL, 5, NULL); // Ajout de la tâche accéléromètre

    ESP_LOGI(TAG, "Système prêt");
}