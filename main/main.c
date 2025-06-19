#include <stdio.h> // incluir biblioteca de entrada e saída padrão
#include "driver/i2c.h" // usar barra de i2c para leitura de sensores
#include "driver/gpio.h" // usar barra de gpio para controle de pinos
#include "esp_log.h" // usar barra de log para mensagens de depuração
#include "freertos/FreeRTOS.h" // usar barra de FreeRTOS para tarefas e filas
#include "freertos/task.h" // usar barra de tarefa para criar tarefas
#include "freertos/queue.h" // usar barra de fila para comunicação entre tarefas
#include "freertos/event_groups.h" // usar barra de eventos para sinalização de eventos
#include "esp_system.h" // usar barra de sistema para funções do sistema
#include <math.h> // usar barra de matemática para cálculos matemáticos

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

mpu6050_data_t imu_data = {0}; // Global variable to hold IMU data
static const char *TAG = "MochilaAntiFurto"; 
float acc_offset_x = 0, acc_offset_y = 0, acc_offset_z = 0; // Calibration offsets for accelerometer
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0; // Calibration offsets for gyroscope

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
                        // Acione alarme aqui
                    } else {
                        ESP_LOGI(TAG, "Movimento descartado. Δ (%d/%d)", contador_movimento, contador_leituras);
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

void  app_main(){
    i2c_master_init(); 
    ESP_LOGI(TAG, "i2c Master initialized");
    mpu6050_init();
    mpu6050_calibrate(100);  // Calibrate IMU

    xTaskCreate(acelerometer_task, "acelerometer_task", 4096, NULL, 5, NULL); 
    ESP_LOGI(TAG, "Acelerometer task started");

}