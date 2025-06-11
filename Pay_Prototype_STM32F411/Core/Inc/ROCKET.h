/*
 * ROCKET.h
 *
 *  Created on: Feb 9, 2025
 *      Author: de4lerr
 */

#ifndef INC_ROCKET_H_
#define INC_ROCKET_H_

#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "quaternion.h"

// Estrutura do filtro de Kalman estendido para estimativa de posição
typedef struct {
    float dt;           // intervalo de tempo (s)
    float x[6];         // vetor de estado: [x, y, z, vx, vy, vz]
    float P[6][6];      // matriz de covariça de estado
    float Q[6][6];      // ruído do processo
    float R_gps[3][3];  // ruído de medição do GPS (para x, y, z)
    float R_baro;       // ruído de medição do barômetro (para z)
} RocketEKF;

typedef struct {
    float dt;           // intervalo de tempo (s)
    Quaternion q;		// quaternion
    float att[3];		// vetor de atitude (phi, theta, gama
    float x[6];         // vetor de estado: [x, y, z, vx, vy, vz]
} Rocket_State;

// Inicializa a estrutura do EKF com parâmetros iniciais
void RocketEKF_Init(RocketEKF *ekf, float dt);

// Passo de predição usando acelerações medidas do IMU
// ax, ay, az: acelerações em m/s²
void RocketEKF_Predict(RocketEKF *ekf, float ax, float ay, float az);

// Atualização com medição do GPS (posição)
// gps_x, gps_y, gps_z: posição estimada pelo GPS (em unidades compatíveis com o sistema, ex. metros)
void RocketEKF_UpdateGPS(RocketEKF *ekf, float gps_x, float gps_y, float gps_z);

// Atualização com medição do barômetro (apenas altitude)
void RocketEKF_UpdateBaro(RocketEKF *ekf, float baro_z);


#endif /* INC_ROCKET_H_ */
