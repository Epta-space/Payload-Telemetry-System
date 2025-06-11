/*
 * GPS.c
 *
 *  Created on: Dec 27, 2024
 *      Author: de4lerr
 */

#include "main.h"
#include "GPS.h"

UART_HandleTypeDef GPSuart;

void GPS_begin(UART_HandleTypeDef uart, GPS *gps)
{
	GPSuart = uart;
}

void GPS_Write(uint8_t data)
{
	HAL_UART_Transmit(&GPSuart, &data, sizeof(data), 1);
}

void GPS_Read(GPS *gps)
{
	HAL_UART_Receive(&GPSuart, gps->GPS_Data, 128, 1);
	//parseNMEA(gps->GPS_Data, gps);
}

// Função que faz o parse da mensagem NMEA e preenche a estrutura GPS
void parseNMEA(char *msg, GPS *gps) {
    char buffer[128];

    // Copia a mensagem para um buffer local, pois strtok modifica a string original
    strncpy(buffer, msg, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *token;

    // 1. Campo: Tipo da mensagem (ex: "$GPRMC")
    token = strtok(buffer, ",");
    if (token != NULL) {
        // Se o primeiro caractere for '$', ele é ignorado
        if (token[0] == '$')
            strncpy(gps->tipo, token + 1, sizeof(gps->tipo) - 1);
        else
            strncpy(gps->tipo, token, sizeof(gps->tipo) - 1);
        gps->tipo[sizeof(gps->tipo) - 1] = '\0';
    }

    // 2. Campo: Hora (hhmmss.ss)
    token = strtok(NULL, ",");
    if (token != NULL) {
        strncpy(gps->hora, token, sizeof(gps->hora) - 1);
        gps->hora[sizeof(gps->hora) - 1] = '\0';
    }

    // 3. Campo: Status (A/V)
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->status = token[0];
    }

    // 4. Campo: Latitude
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->lat = atof(token);
    }

    // 5. Campo: Direção da Latitude (N ou S)
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->lat_dir = token[0];
    }

    // 6. Campo: Longitude
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->lon = atof(token);
    }

    // 7. Campo: Direção da Longitude (E ou W)
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->lon_dir = token[0];
    }

    // 8. Campo: Velocidade (em nós)
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->velocidade = atof(token);
    }

    // 9. Campo: Ângulo (cursos)
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->angulo = atof(token);
    }

    // 10. Campo: Data (ddmmyy)
    token = strtok(NULL, ",");
    if (token != NULL) {
        strncpy(gps->data, token, sizeof(gps->data) - 1);
        gps->data[sizeof(gps->data) - 1] = '\0';
    }

    // 11. Campo: Variação magnética
    token = strtok(NULL, ",");
    if (token != NULL) {
        gps->variacao_magnetica = atof(token);
    }

    // 12. Campo: Direção da variação magnética
    token = strtok(NULL, ",");
    if (token != NULL) {
        // Caso o campo contenha o caractere '*' (checksum), remova-o.
        char *asterisco = strchr(token, '*');
        if (asterisco != NULL) {
            *asterisco = '\0';
        }
        gps->direcao_variacao = token[0];
    }
}
