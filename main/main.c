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

// Phone number to send SMS
#define TELEFONE_NUMBER "\"+5511939069320\""

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
volatile bool gsm_network_registered = false;

// Protótipos
static void IRAM_ATTR btn_emerg_isr_handler(void *arg);
static void IRAM_ATTR btn_antifurto_isr_handler(void *arg);
void send_sms(const char *message);
void buzzer_on();
void buzzer_off();

void gsm_network_task(void *pvParameters) {
    char buffer[128];
    int len;

    while (1) {
        uart_write_bytes(GSM_UART, "AT+CREG?\r", 9);
        len = uart_read_bytes(GSM_UART, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI("GSM", "CREG: %s", buffer);

            if (strstr(buffer, "+CREG: 0,1") || strstr(buffer, "+CREG: 0,5")) {
                gsm_network_registered = true;
                ESP_LOGI("GSM", "Registrado na rede GSM!");
            } else {
                gsm_network_registered = false;
                ESP_LOGW("GSM", "Ainda procurando rede...");
            }
        } else {
            ESP_LOGW("GSM", "Sem resposta do módulo");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
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

void gsm_init() {
    const uart_config_t gsm_config = {
        .baud_rate = GSM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(GSM_UART, 2048, 0, 0, NULL, 0);
    uart_param_config(GSM_UART, &gsm_config);
    uart_set_pin(GSM_UART, GSM_TXD, GSM_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    char buffer[128];

    // Testa comunicação básica com AT
    for (int i = 0; i < 3; i++) {
        uart_write_bytes(GSM_UART, "AT\r", 3);
        int len = uart_read_bytes(GSM_UART, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            buffer[len] = '\0';
            if (strstr(buffer, "OK")) {
                ESP_LOGI("GSM", "Comunicação AT OK");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Verifica o estado do chip SIM
    uart_write_bytes(GSM_UART, "AT+CPIN?\r", 9);
    int len = uart_read_bytes(GSM_UART, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(1000));
    if (len <= 0 || !strstr(buffer, "READY")) {
        ESP_LOGE("GSM", "Chip SIM não pronto ou PIN ativo");
        return;
    }

    // Tenta se registrar na rede por até 30 segundos
    for (int i = 0; i < 15; i++) {
        uart_write_bytes(GSM_UART, "AT+CREG?\r", 9);
        len = uart_read_bytes(GSM_UART, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI("GSM", "CREG: %s", buffer);
            if (strstr(buffer, "+CREG: 0,1") || strstr(buffer, "+CREG: 0,5")) {
                ESP_LOGI("GSM", "Registrado na rede celular");
                return;
            }
        }
        ESP_LOGW("GSM", "Aguardando registro na rede (%d)...", i + 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGE("GSM", "Falha ao registrar na rede após várias tentativas");
    return;
}




void gsm_read_response() {
    uint8_t resp[128];
    int len = uart_read_bytes(GSM_UART, resp, sizeof(resp) - 1, pdMS_TO_TICKS(1000));
    if (len > 0) {
        resp[len] = '\0';
        ESP_LOGI("GSM", "Resposta: %s", (char *)resp);
    } else {
        ESP_LOGW("GSM", "Nenhuma resposta recebida");
    }
}


bool gsm_wait_for_network() {
    for (int i = 0; i < 10; i++) {
        uart_write_bytes(GSM_UART, "AT+CREG?\r", 9);
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint8_t resp[128] = {0};
        int len = uart_read_bytes(GSM_UART, resp, sizeof(resp) - 1, pdMS_TO_TICKS(500));
        if (len > 0) {
            resp[len] = '\0';
            ESP_LOGI("GSM", "CREG: %s", (char *)resp);
            if (strstr((char *)resp, "+CREG: 0,1") || strstr((char *)resp, "+CREG: 0,5")) {
                return true;
            }
        }
    }
    ESP_LOGE("GSM", "Falha ao registrar na rede");
    return false;
}

bool gsm_wait_for_prompt() {
    uint8_t resp[128] = {0};
    int len = uart_read_bytes(GSM_UART, resp, sizeof(resp) - 1, pdMS_TO_TICKS(3000));
    if (len > 0) {
        resp[len] = '\0';
        ESP_LOGI("GSM", "Prompt: %s", (char *)resp);
        return strchr((char *)resp, '>') != NULL;
    }
    return false;
}

void send_sms(const char *message) {
    const char *numero = TELEFONE_NUMBER;
    char cmd[100];

    uart_write_bytes(GSM_UART, "AT\r", 3);
    gsm_read_response();

    uart_write_bytes(GSM_UART, "AT+CSQ\r", 8);
    gsm_read_response();

    if (!gsm_wait_for_network()) {
        ESP_LOGE("GSM", "Abortando envio de SMS - sem rede");
        return;
    }

    uart_write_bytes(GSM_UART, "AT+CMGF=1\r", 10);
    gsm_read_response();

    snprintf(cmd, sizeof(cmd), "AT+CMGS=%s\r", numero);
    uart_write_bytes(GSM_UART, cmd, strlen(cmd));

    if (!gsm_wait_for_prompt()) {
        ESP_LOGE("GSM", "Não recebeu o prompt '>' para envio de mensagem");
        return;
    }

    uart_write_bytes(GSM_UART, message, strlen(message));
    uart_write_bytes(GSM_UART, "\x1A", 1);  // Ctrl+Z
    gsm_read_response();

    ESP_LOGI("GSM", "SMS enviado: %s", message);
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
                send_sms("Alerte : ouverture détectée !");
                estado_global = ALARMING;
                continue;
            }
            // Vérification du mouvement détecté par acelerometer_task
            if (mouvement_detecte) {
                ESP_LOGW(TAG, "Alerte : mouvement suspect détecté !");
                buzzer_on();
                send_sms("Alerte : mouvement suspect détecté !");
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
            send_sms(gps_position);
            ESP_LOGI(TAG, "SMS urgence envoyé avec position GPS.");
            // Attendre 3 minutes (180000 ms)
            vTaskDelay(pdMS_TO_TICKS(9000));
            buzzer_on();
            ESP_LOGI(TAG, "Buzzer activé en mode urgence.");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void gsm_list_available_operators() {
    char buffer[512];  // pode vir uma resposta grande!
    ESP_LOGI("GSM", "Solicitando lista de operadoras...");

    uart_write_bytes(GSM_UART, "AT+COPS=?\r", 10);

    int len = uart_read_bytes(GSM_UART, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(30000));  // até 30s
    if (len > 0) {
        buffer[len] = '\0';
        ESP_LOGI("GSM", "Resposta COPS: %s", buffer);
    } else {
        ESP_LOGW("GSM", "Sem resposta do COPS (timeout)");
    }
}

void gsm_check_signal() {
    char buffer[128];
    int len;

    // Envia comando AT+CSQ
    uart_write_bytes(GSM_UART, "AT+CSQ\r", 7);

    // Lê resposta com timeout de 2 segundos
    len = uart_read_bytes(GSM_UART, (uint8_t *)buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(2000));
    if (len > 0) {
        buffer[len] = '\0';
        ESP_LOGI("GSM", "Resposta AT+CSQ: %s", buffer);

        // Opcional: extrair o valor do sinal do buffer
        int rssi = -1;
        if (sscanf(buffer, "\r\n+CSQ: %d,", &rssi) == 1) {
            ESP_LOGI("GSM", "Qualidade do sinal (RSSI): %d", rssi);
            if (rssi == 99) {
                ESP_LOGW("GSM", "Sinal desconhecido ou fora de alcance");
            } else if (rssi >= 10) {
                ESP_LOGI("GSM", "Sinal aceitável");
            } else {
                ESP_LOGW("GSM", "Sinal fraco");
            }
        } else {
            ESP_LOGW("GSM", "Não conseguiu parsear o RSSI");
        }
    } else {
        ESP_LOGW("GSM", "Timeout lendo resposta AT+CSQ");
    }
}


void app_main() {
    i2c_master_init(); 
    ESP_LOGI(TAG, "i2c Master initialized");
    mpu6050_init();
    mpu6050_calibrate(100);
    gps_init();
    gsm_init();
    botoes_init();
    //uart_write_bytes(GSM_UART, "AT+COPS=1,2,\"72405\"\r", strlen("AT+COPS=1,2,\"72405\"\r"));
    gsm_check_signal();
    gsm_list_available_operators();
    xTaskCreate(gsm_network_task, "GSM Network Monitor", 4096, NULL, 5, NULL);

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(botoes_task, "botoes_task", 2048, NULL, 5, NULL);
    xTaskCreate(antifurto_task, "antifurto_task", 4096, NULL, 6, NULL);
    xTaskCreate(emergencia_task, "emergencia_task", 2048, NULL, 7, NULL);
    xTaskCreate(acelerometer_task, "acelerometer_task", 4096, NULL, 5, NULL); // Ajout de la tâche accéléromètre

    ESP_LOGI(TAG, "Système prêt");
}