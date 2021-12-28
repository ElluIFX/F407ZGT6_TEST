/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef MPU6050_H_
#define MPU6050_H_
#include <stdint.h>

#include "i2c.h"

// MPU6050 structure
typedef struct {        // MPU6050 data struct
  int16_t Accel_X_RAW;  // X-axis accelerometer raw value
  int16_t Accel_Y_RAW;  // Y-axis accelerometer raw value
  int16_t Accel_Z_RAW;  // Z-axis accelerometer raw value
  double Ax;            // X-axis acceleration
  double Ay;            // Y-axis acceleration
  double Az;            // Z-axis acceleration

  int16_t Gyro_X_RAW;  // X-axis gyroscope raw value
  int16_t Gyro_Y_RAW;  // Y-axis gyroscope raw value
  int16_t Gyro_Z_RAW;  // Z-axis gyroscope raw value
  double Gx;           // X-axis angular velocity
  double Gy;           // Y-axis angular velocity
  double Gz;           // Z-axis angular velocity

  float Temperature;  // Temperature

  double KalmanAngleX;      // Kalman filtering X-axis angle
  double KalmanAngleY;      // Kalman filtering Y-axis angle
  double ZDegree;           // Z-axis degree
  double ZDzeroError;       // Z-axis degree zero error
  double sumZeroError;      // Sum of Z-axis degree zero error
  int16_t getErrorCount;    // count sum of Z-axis degree zero error
  int8_t getErrorFlag;      // get error flag
  I2C_HandleTypeDef *I2Cx;  // I2Cx
} MPU6050_t;

// Kalman structure
typedef struct {     // Kalman data struct
  double Q_angle;    // Process noise variance for the accelerometer
  double Q_bias;     // Process noise variance for the gyro bias
  double R_measure;  // Measurement noise variance - this is actually the
                     // variance of the measurement noise
  double angle;  // The angle calculated by the Kalman filter - part of the 2x1
                 // matrix A
  double bias;   // The gyro bias calculated by the Kalman filter - part of the
                 // 2x1 matrix B
  double P[2][2];  // Error covariance matrix - part of 2x2 matrix P
} Kalman_t;

uint8_t MPU6050_Init(MPU6050_t *DataStruct, I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(MPU6050_t *DataStruct);

void MPU6050_Read_Temp(MPU6050_t *DataStruct);

void MPU6050_Read_All(MPU6050_t *DataStruct);

void MPU6050_Start_Calibration(MPU6050_t *DataStruct);

void MPU6050_Stop_Calibration(MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate,
                       double dt);

#endif /* MPU6050_H_ */