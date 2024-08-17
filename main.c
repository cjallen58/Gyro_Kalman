/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*############ Libraries ############*/

#include <stdio.h>
#include <time.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/*############ Defines ############*/

// I2C
#define I2C_PORT i2c0
#define MPU6050_ADDR 0x68
#define PICO_SDA_PIN 4
#define PICO_SCL_PIN 5

// MPU6050 registers
#define MPU6050_PWR_CTRL_1 0x6B
#define MPU6050_GYRO_CONFIG 0X1B
#define MPU6050_ACCEL_CONFIG 0X1C
#define MPU6050_GYRO_READINGS 0x43
#define MPU6050_ACCEL_READINGS 0x3B
#define MPU6050_TEMP_READING 0x41
#define MPU6050_WHOAMI_REG 0x75

// Gyro and accelerometer configurations
#define MPU6050_GYRO_250DPS 0b00000000
#define MPU6050_GYRO_500DPS 0b00001000
#define MPU6050_GYRO_1000DPS 0b00010000
#define MPU6050_GYRO_2000DPS 0b00011000

#define MPU6050_ACCEL_2G 0b00000000
#define MPU6050_ACCEL_4G 0b00001000
#define MPU6050_ACCEL_8G 0b00010000
#define MPU6050_ACCEL_16G 0b00011000

// Gyro and accelerometer scale factors
#define GYRO_250DPS_FACTOR 131.0
#define GYRO_500DPS_FACTOR 65.5
#define GYRO_1000DPS_FACTOR 32.8
#define GYRO_2000DPS_FACTOR 16.4

#define ACCEL_2G_FACTOR 16384.0
#define ACCEL_4G_FACTOR 8192.0
#define ACCEL_8G_FACTOR 4096.0
#define ACCEL_16G_FACTOR 2048.0


// Power control values
#define MPU6050_PWR_CTRL_RESET 0b10000000
#define MPU6050_PWR_CTRL_WAKE 0b00000000

/*############ Srtucts ############*/

typedef struct {
    // The predicted from gyro data
    float predicted_roll; 
    float predicted_pitch;
    // Original gyro predictions
    float gyro_roll;
    float gyro_pitch;
    // measured from Accelerometer
    float measured_roll;  
    float measured_pitch;
} KalmanState;

typedef struct {
    float P[2][2]; // Error covariance matrix
    float Q[2][2]; // Process noise variance for the accelerometer
    float R[2][2]; // Measurement noise variance
    float K[2][2]; // Kalman gain
   
} KalmanFilter;

/*############ Function Definitions ############*/

// MPU6050 reset
// call this on start up
static void mpu6050_reset() {

    uint8_t reg;
    uint8_t buf[] = {MPU6050_PWR_CTRL_1, MPU6050_PWR_CTRL_RESET};
    uint8_t whoami;
    
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    sleep_ms(200);

    buf[1] = MPU6050_PWR_CTRL_WAKE;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    sleep_ms(200);

    reg = MPU6050_WHOAMI_REG;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &whoami, 1, false);

    if (whoami != 0x68){
        printf("Incorrect device at address. Who Am I reads: 0x%2x\n", whoami);
        
        while (1);
    }
}

// MPU6050 set configs
static void mpu6050_config(uint8_t gyro_config, uint8_t accel_config) {

    //set gyro config
    uint8_t buf[] = {MPU6050_GYRO_CONFIG, gyro_config};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    //set accelerometer congif
    buf[0] = MPU6050_ACCEL_CONFIG;
    buf[1] = accel_config;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

}

//MPU readdata
static void mpu6050_data_read(int16_t gyro[3], int16_t accel[3], int16_t *temp) {
    
    uint8_t buf[6];
    uint8_t reg;
   //read gyro
    reg = MPU6050_GYRO_READINGS;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 6, false);

    //combine bytes and store in gyro array
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    }

    //read accel
    reg = MPU6050_ACCEL_READINGS;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 6, false);

    //combine bytes and store in accel array
    for (int i = 0; i < 3; i++) {
        accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]);
    }

    //read temp
    reg = MPU6050_TEMP_READING;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);  

    //combine bytes and store
    *temp = buf[0] << 8 | buf[1];

}

//Calibrate MPU
static void mpu6050_calibrate(int32_t gyro_cal[3], int32_t accel_cal[3]) {
    /*arrays are 32 bit to avoid potential for overflow*/

    int16_t gyro[3] = {0}, accel[3] = {0}, temp ;

    //initial time to settle
    sleep_ms(300);
    printf("Gathering measurements for calibration\n");

    // capture a number of readings and sum them for each axis 
    for (uint8_t i = 0; i < 100; i++) {
        mpu6050_data_read(gyro, accel, &temp);

        gyro_cal[0] += gyro[0];
        gyro_cal[1] += gyro[1];
        gyro_cal[2] += gyro[2];

        accel_cal[0] += accel[0];
        accel_cal[1] += accel[1];
        accel_cal[2] += accel[2];

        //sleep to give some buffer between measurements
        sleep_ms(100);
        
        if (i % 5 == 0 || i == 0){

            printf("values: Ax: %i, Ay: %i, Az: %i, Gx: %i, Gy: %i, Gz: %i\n",
                    accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
            
            printf("sum values: Ax: %i, Ay: %i, Az: %i, Gx: %i, Gy: %i, Gz: %i\n",
                    accel_cal[0], accel_cal[1], accel_cal[3], gyro_cal[0], gyro_cal[1], gyro_cal[2]);

        }
    }

    //averaging the measurements
    for (uint8_t i = 0; i < 3; i++) {

        gyro_cal[i] /= 100;
        accel_cal[i] /= 100;

    }
    //account for gravity in calibration
    accel_cal[2] -= (1 * ACCEL_8G_FACTOR);

    printf("Calibration complete\n");
    printf("values: Ax: %i, Ay: %i, Az: %i, Gx: %i, Gy: %i, Gz: %i\n",
            accel_cal[0], accel_cal[1], accel_cal[2], gyro_cal[0], gyro_cal[1], gyro_cal[2]);
    
    printf("values: Ax: %2f, Ay: %2f, Az: %2f, Gx: %2f, Gy: %2f, Gz: %2f\n",
        (float) (accel_cal[0] / ACCEL_8G_FACTOR), 
        (float) (accel_cal[1] / ACCEL_8G_FACTOR),
        (float) (accel_cal[2] / ACCEL_8G_FACTOR), 
        (float) (gyro_cal[0] / GYRO_500DPS_FACTOR), 
        (float) (gyro_cal[1] / GYRO_500DPS_FACTOR),
        (float) (gyro_cal[2] / GYRO_500DPS_FACTOR)
        );
    sleep_ms(5000);        
}

// Kalman filter initialization
void KalmanInit(KalmanState *state, KalmanFilter *filter) {  
        //initial state
        state->predicted_roll = 0.0f;
        state->predicted_pitch = 0.0f;
        state->measured_roll = 0.0f;
        state->measured_pitch = 0.0f;

        //initial error covariance matrix
        filter->P[0][0] = 0.1f;
        filter->P[0][1] = 0.0f;
        filter->P[1][0] = 0.0f;
        filter->P[1][1] = 0.1f;

        //initial process noise covariance matrix
        filter->Q[0][0] = 0.001f;
        filter->Q[0][1] = 0.000f;
        filter->Q[1][0] = 0.000f;
        filter->Q[1][1] = 0.001f;

        //initial Measurement Noise Covariance Matrix
        filter->R[0][0] = 0.05f;
        filter->R[0][1] = 0.00f;
        filter->R[1][0] = 0.00f;
        filter->R[1][1] = 0.05f;
}

void KalmanUpdate(KalmanState *state, KalmanFilter *filter, float newAngle[2], float newRate[3], float dt) {
    // Step 1 Predict
    // State prediction with gyro measurements
    state->predicted_roll +=  newRate[0] * dt;
    state->predicted_pitch += newRate[1] * dt;

    state->gyro_roll = state->predicted_roll;
    state->gyro_pitch = state->predicted_pitch;
    
    // Error Covariance predicition 
    filter->P[0][0] += filter->Q[0][0];
    filter->P[1][1] += filter->Q[1][1];

    state->measured_roll = newAngle[0];
    state->measured_pitch = newAngle[1];


    // Step 2 Update
    // Gain update
    filter->K[0][0] = filter->P[0][0] / (filter->P[0][0] + filter->R[0][0]);
    filter->K[1][1] = filter->P[1][1] / (filter->P[1][1] + filter->R[1][1]);

    // state update with gain and measured roll
    state->predicted_roll += filter->K[0][0] * (state->measured_roll - state->predicted_roll);
    state->predicted_pitch += filter->K[1][1] * (state->measured_pitch - state->predicted_pitch);

    filter->P[0][0] = (1 - filter->K[0][0]) * filter->P[0][0];
    filter->P[1][1] = (1 - filter->K[1][1]) * filter->P[1][1];
}

/*############ main ############*/

int main() {
    //initialize and delay setup so I can connect
    stdio_init_all();

    sleep_ms(5000);

    for (uint8_t i = 5; i > 0; i--){
        printf("%i\n",i);
        sleep_ms(1000);
    }

    //set up i2c
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PICO_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_SDA_PIN);
    gpio_pull_up(PICO_SCL_PIN);

    //reset mpu6050
    mpu6050_reset();
    sleep_ms(100);
    //set mpu6050 config
    mpu6050_config(MPU6050_GYRO_500DPS, MPU6050_ACCEL_8G);

    //calibrate mpu6050
    int32_t gyro_cal[3] = {0}, accel_cal[3] = {0};
    mpu6050_calibrate(gyro_cal, accel_cal);


    //arrays for sensor readings
    int16_t raw_accel[3], raw_gyro[3], temp;
    clock_t time_start;
    clock_t time_end;
    
    //Kalman filter
    KalmanState kalmanState;
    KalmanFilter kalmanFilter;
    KalmanInit(&kalmanState, &kalmanFilter);
    
    //loop for readings
    while(1) {
        const float dt = 0.01f;

        mpu6050_data_read(raw_gyro, raw_accel, &temp);

        float accel_g[3];
        accel_g[0] = (raw_accel[0] - accel_cal[0]) / ACCEL_8G_FACTOR;
        accel_g[1] = (raw_accel[1] - accel_cal[1]) / ACCEL_8G_FACTOR;
        accel_g[2] = (raw_accel[2] - accel_cal[2]) / ACCEL_8G_FACTOR;
        
        float gyro_dps[3];
        gyro_dps[0] = (raw_gyro[0] - gyro_cal[0]) / GYRO_500DPS_FACTOR;
        gyro_dps[1] = (raw_gyro[1] - gyro_cal[1]) / GYRO_500DPS_FACTOR;
        gyro_dps[2] = (raw_gyro[2] - gyro_cal[2]) / GYRO_500DPS_FACTOR;

        
        double tempC = temp / 340.00 + 36.53;
        
        printf("aX = %.2f g | aY = %.2f g | aZ = %.2f g | gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | temp = %.2f°C\n",
            accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], tempC);

        
        //calculate angle from acceleration
        float accel_angle[2] = {
            (180 / 3.14159) * atan2f(accel_g[1], sqrt(accel_g[2] * accel_g[2] + accel_g[0] * accel_g[0])),
            (180 / 3.14159) * atan2f(accel_g[0], sqrtf(accel_g[1] * accel_g[1] + accel_g[2]* accel_g[2])),
        };

        KalmanUpdate(&kalmanState, &kalmanFilter, accel_angle, gyro_dps, dt);
        
        //printf("accel_angle[0]: %.2f, accel_angle[1]: %.2f\n", accel_angle[0], accel_angle[1]);
        printf("Measured Roll: %.2f, Measured Pitch: %.2f\n", kalmanState.measured_roll, kalmanState.measured_pitch);
        printf("Predicted Roll: %.2f, Predicted Pitch: %.2f\n", kalmanState.gyro_roll, kalmanState.gyro_pitch );    
        printf("Filtered Roll: %.2f, Filtered Pitch: %.2f\n\n", kalmanState.predicted_roll, kalmanState.predicted_pitch);

        sleep_ms(10);
    } 

}