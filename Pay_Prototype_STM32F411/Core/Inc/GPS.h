/*
 * GPS.h
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct {
	char GPS_Data[128];
    char tipo[6];               // Ex: "GPRMC"
    char hora[11];              // Hora (hhmmss.ss)
    char status;                // Status (A = ativo, V = inválido)
    double lat;                 // Latitude (graus e minutos, ex: 4807.038)
    char lat_dir;               // Direção da latitude ('N' ou 'S')
    double lon;                 // Longitude (ex: 01131.000)
    char lon_dir;               // Direção da longitude ('E' ou 'W')
    double velocidade;          // Velocidade em nós
    double angulo;              // Ângulo (cursos)
    char data[7];               // Data (ddmmyy)
    double variacao_magnetica;  // Variação magnética
    char direcao_variacao;      // Direção da variação magnética ('E' ou 'W')
} GPS;

void GPS_begin(UART_HandleTypeDef uart, GPS *gps);
void GPS_Write(uint8_t data);
void GPS_Read(GPS *gps);

void parseNMEA(char *msg, GPS *gps);


#endif /* INC_GPS_H_ */
