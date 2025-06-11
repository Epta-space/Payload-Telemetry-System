/*
 * ROCKET.c
 *
 *  Created on: Feb 9, 2025
 *      Author: de4lerr
 */

#include "ROCKET.h"
#include <math.h>
#include <string.h>

// Função auxiliar para inverter uma matriz 3x3
// Retorna 1 se a inversão for bem-sucedida, 0 caso contrário.
static int invert3x3(const float A[3][3], float A_inv[3][3]) {
    float det = A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
              - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
              + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabs(det) < 1e-6) return 0; // Determinante muito pequeno, não inversível

    A_inv[0][0] =  (A[1][1]*A[2][2]-A[1][2]*A[2][1]) / det;
    A_inv[0][1] = -(A[0][1]*A[2][2]-A[0][2]*A[2][1]) / det;
    A_inv[0][2] =  (A[0][1]*A[1][2]-A[0][2]*A[1][1]) / det;

    A_inv[1][0] = -(A[1][0]*A[2][2]-A[1][2]*A[2][0]) / det;
    A_inv[1][1] =  (A[0][0]*A[2][2]-A[0][2]*A[2][0]) / det;
    A_inv[1][2] = -(A[0][0]*A[1][2]-A[0][2]*A[1][0]) / det;

    A_inv[2][0] =  (A[1][0]*A[2][1]-A[1][1]*A[2][0]) / det;
    A_inv[2][1] = -(A[0][0]*A[2][1]-A[0][1]*A[2][0]) / det;
    A_inv[2][2] =  (A[0][0]*A[1][1]-A[0][1]*A[1][0]) / det;

    return 1;
}

void RocketEKF_Init(RocketEKF *ekf, float dt) {
    int i, j;
    ekf->dt = dt;
    // Inicializa o vetor de estado com zero
    for(i = 0; i < 6; i++) {
        ekf->x[i] = 0.0f;
    }
    // Inicializa P como identidade multiplicada por uma incerteza inicial (ex.: 1.0)
    memset(ekf->P, 0, sizeof(ekf->P));
    for(i = 0; i < 6; i++) {
        ekf->P[i][i] = 1.0f;
    }
    // Define o ruído do processo Q (ajuste conforme o sistema)
    memset(ekf->Q, 0, sizeof(ekf->Q));
    for(i = 0; i < 3; i++) { // para posições
        ekf->Q[i][i] = 0.01f;
    }
    for(i = 3; i < 6; i++) { // para velocidades
        ekf->Q[i][i] = 0.1f;
    }
    // Define o ruído de medição do GPS (matriz 3x3, valores exemplo)
    memset(ekf->R_gps, 0, sizeof(ekf->R_gps));
    ekf->R_gps[0][0] = 5.0f;
    ekf->R_gps[1][1] = 5.0f;
    ekf->R_gps[2][2] = 5.0f;
    // Ruído do barômetro (apenas para z)
    ekf->R_baro = 2.0f;
}

void RocketEKF_Predict(RocketEKF *ekf, float ax, float ay, float az) {
    int i, j, k;
    float dt = ekf->dt;
    float dt2 = 0.5f * dt * dt;

    // Predição do estado com base na dinâmica:
    // x = x + vx*dt + 0.5*a*dt^2 ; vx = vx + a*dt
    ekf->x[0] += ekf->x[3]*dt + dt2 * ax;
    ekf->x[1] += ekf->x[4]*dt + dt2 * ay;
    ekf->x[2] += ekf->x[5]*dt + dt2 * az;
    ekf->x[3] += dt * ax;
    ekf->x[4] += dt * ay;
    ekf->x[5] += dt * az;

    // Matriz de transição de estado F (6x6)
    float F[6][6] = {
      {1, 0, 0, dt, 0,  0},
      {0, 1, 0, 0,  dt, 0},
      {0, 0, 1, 0,  0,  dt},
      {0, 0, 0, 1,  0,  0},
      {0, 0, 0, 0,  1,  0},
      {0, 0, 0, 0,  0,  1}
    };

    // Atualiza a covariância: P = F*P*F^T + Q
    float P_temp[6][6] = {0};

    // Cálculo: P_temp = F * P
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            for(k = 0; k < 6; k++) {
                P_temp[i][j] += F[i][k] * ekf->P[k][j];
            }
        }
    }

    float P_new[6][6] = {0};
    // Cálculo: P_new = P_temp * F^T
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            for(k = 0; k < 6; k++) {
                P_new[i][j] += P_temp[i][k] * F[j][k]; // note que F^T[j][k] = F[k][j]
            }
            // Soma o ruído do processo
            P_new[i][j] += ekf->Q[i][j];
        }
    }
    // Atualiza P
    memcpy(ekf->P, P_new, sizeof(P_new));
}

void RocketEKF_UpdateGPS(RocketEKF *ekf, float gps_x, float gps_y, float gps_z) {
    int i, j, k;
    // Vetor de medição z (GPS) e predição h(x) = [x, y, z]
    float z_meas[3] = { gps_x, gps_y, gps_z };
    float z_pred[3] = { ekf->x[0], ekf->x[1], ekf->x[2] };

    // Inovação: y = z - h(x)
    float y_innov[3] = { z_meas[0]-z_pred[0],
                         z_meas[1]-z_pred[1],
                         z_meas[2]-z_pred[2] };

    // Matriz de medição H (3x6): apenas as 3 primeiras linhas da identidade
    // Assim, H * P * H^T resulta no bloco P[0..2][0..2]
    float S[3][3] = {0};
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            S[i][j] = ekf->P[i][j] + ekf->R_gps[i][j];
        }
    }

    // Inverte a matriz S (3x3)
    float S_inv[3][3];
    if (!invert3x3(S, S_inv)) {
        // Se não conseguir inverter, aborta a atualização
        return;
    }

    // Calcula o ganho de Kalman: K = P * H^T * S_inv
    // Como H é 3x6 (seleciona as 3 primeiras variáveis) temos:
    // K terá dimensão 6x3: K[i][j] = soma_{k=0}^{2} P[i][k] * S_inv[k][j]
    float K[6][3] = {0};
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 3; j++) {
            for(k = 0; k < 3; k++) {
                K[i][j] += ekf->P[i][k] * S_inv[k][j];
            }
        }
    }

    // Atualiza o estado: x = x + K * y_innov
    for(i = 0; i < 6; i++) {
        float delta = 0;
        for(j = 0; j < 3; j++) {
            delta += K[i][j] * y_innov[j];
        }
        ekf->x[i] += delta;
    }

    // Atualiza a covariância: P = (I - K*H) * P
    float P_new[6][6];
    // Inicializa P_new com os valores de P
    memcpy(P_new, ekf->P, sizeof(P_new));
    // Atualiza: para cada i,j, subtrai K[i][k] * P[k][j] com k variando de 0 a 2.
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            float correction = 0;
            for(k = 0; k < 3; k++) {
                correction += K[i][k] * ekf->P[k][j];
            }
            P_new[i][j] -= correction;
        }
    }
    memcpy(ekf->P, P_new, sizeof(P_new));
}

void RocketEKF_UpdateBaro(RocketEKF *ekf, float baro_z) {
    int i, j;
    // Medição do barômetro: apenas altitude (z)
    float z_meas = baro_z;
    float z_pred = ekf->x[2];
    float y = z_meas - z_pred; // Inovação

    // Matriz H para barômetro: [0, 0, 1, 0, 0, 0]
    // S = H * P * H^T + R_baro = P[2][2] + R_baro
    float S = ekf->P[2][2] + ekf->R_baro;
    if (fabs(S) < 1e-6) return;

    // Ganho de Kalman K (vetor de 6 elementos)
    float K[6];
    for(i = 0; i < 6; i++) {
        K[i] = ekf->P[i][2] / S;
    }

    // Atualiza o estado: x = x + K * y
    for(i = 0; i < 6; i++) {
        ekf->x[i] += K[i] * y;
    }

    // Atualiza a covariância: P = P - K * H * P, ou para cada i,j:
    // P[i][j] = P[i][j] - K[i] * P[2][j]
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 6; j++) {
            ekf->P[i][j] -= K[i] * ekf->P[2][j];
        }
    }
}
