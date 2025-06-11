/*
 * IMU.h
 *
 *  Created on: Jun 28, 2024
 *      Author: diler
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "quaternion.h"

typedef enum ACC_RANGE {
    ACC_RANGE_2G = 0b00<<1,
	ACC_RANGE_4G = 0b01<<1,
	ACC_RANGE_8G = 0b10<<1,
	ACC_RANGE_16G = 0b11<<1
} accRange;

typedef enum Gyro_RANGE {
    GYRO_RANGE_250DPS = 0b00<<1,
	GYRO_RANGE_500DPS = 0b01<<1,
	GYRO_RANGE_1000DPS = 0b10<<1,
	GYRO_RANGE_2000DPS = 0b11<<1
} gyroRange;

static const uint8_t AK09916_ADDRESS =             {0x0C};

/* Registers ICM20948 USER BANK 0*/
static const uint8_t ICM20948_WHO_AM_I =           {0x00};
static const uint8_t ICM20948_USER_CTRL =          {0x03};
static const uint8_t ICM20948_LP_CONFIG =          {0x05};
static const uint8_t ICM20948_PWR_MGMT_1 =         {0x06};
static const uint8_t ICM20948_PWR_MGMT_2=          {0x07};
static const uint8_t ICM20948_INT_PIN_CFG=         {0x0F};
static const uint8_t ICM20948_INT_ENABLE=          {0x10};
static const uint8_t ICM20948_INT_ENABLE_1=        {0x11};
static const uint8_t ICM20948_INT_ENABLE_2=        {0x12};
static const uint8_t ICM20948_INT_ENABLE_3=        {0x13};
static const uint8_t ICM20948_I2C_MST_STATUS=      {0x17};
static const uint8_t ICM20948_INT_STATUS=          {0x19};
static const uint8_t ICM20948_INT_STATUS_1=        {0x1A};
static const uint8_t ICM20948_INT_STATUS_2=        {0x1B};
static const uint8_t ICM20948_INT_STATUS_3=        {0x1C};
static const uint8_t ICM20948_DELAY_TIME_H=        {0x28};
static const uint8_t ICM20948_DELAY_TIME_L=        {0x29};
static const uint8_t ICM20948_ACCEL_OUT=           {0x2D}; // accel data registers begin
static const uint8_t ICM20948_GYRO_OUT=            {0x33}; // gyro data registers begin
static const uint8_t ICM20948_TEMP_OUT=            {0x39};
static const uint8_t ICM20948_EXT_SLV_SENS_DATA_00={0x3B};
static const uint8_t ICM20948_EXT_SLV_SENS_DATA_01={0x3C};
static const uint8_t ICM20948_FIFO_EN_1=           {0x66};
static const uint8_t ICM20948_FIFO_EN_2=           {0x67};
static const uint8_t ICM20948_FIFO_RST=            {0x68};
static const uint8_t ICM20948_FIFO_MODE=           {0x69};
static const uint8_t ICM20948_FIFO_COUNT=          {0x70};
static const uint8_t ICM20948_FIFO_R_W=            {0x72};
static const uint8_t ICM20948_DATA_RDY_STATUS=     {0x74};
static const uint8_t ICM20948_FIFO_CFG=            {0x76};
/* Registers ICM20948 USER BANK 1*/
static const uint8_t ICM20948_SELF_TEST_X_GYRO=    {0x02};
static const uint8_t ICM20948_SELF_TEST_Y_GYRO=    {0x03};
static const uint8_t ICM20948_SELF_TEST_Z_GYRO=    {0x04};
static const uint8_t ICM20948_SELF_TEST_X_ACCEL=   {0x0E};
static const uint8_t ICM20948_SELF_TEST_Y_ACCEL=   {0x0F};
static const uint8_t ICM20948_SELF_TEST_Z_ACCEL=   {0x10};
static const uint8_t ICM20948_XA_OFFS_H=           {0x14};
static const uint8_t ICM20948_XA_OFFS_L=           {0x15};
static const uint8_t ICM20948_YA_OFFS_H=           {0x17};
static const uint8_t ICM20948_YA_OFFS_L=           {0x18};
static const uint8_t ICM20948_ZA_OFFS_H=           {0x1A};
static const uint8_t ICM20948_ZA_OFFS_L=           {0x1B};
static const uint8_t ICM20948_TIMEBASE_CORR_PLL=   {0x28};

/* Registers ICM20948 USER BANK 2*/
static const uint8_t ICM20948_GYRO_SMPLRT_DIV=     {0x00};
static const uint8_t ICM20948_GYRO_CONFIG_1=       {0x01};
static const uint8_t ICM20948_GYRO_CONFIG_2=       {0x02};
static const uint8_t ICM20948_XG_OFFS_USRH=        {0x03};
static const uint8_t ICM20948_XG_OFFS_USRL=        {0x04};
static const uint8_t ICM20948_YG_OFFS_USRH=        {0x05};
static const uint8_t ICM20948_YG_OFFS_USRL=        {0x06};
static const uint8_t ICM20948_ZG_OFFS_USRH=        {0x07};
static const uint8_t ICM20948_ZG_OFFS_USRL=        {0x08};
static const uint8_t ICM20948_ODR_ALIGN_EN=        {0x09};
static const uint8_t ICM20948_ACCEL_SMPLRT_DIV_1=  {0x10};
static const uint8_t ICM20948_ACCEL_SMPLRT_DIV_2=  {0x11};
static const uint8_t ICM20948_ACCEL_INTEL_CTRL=    {0x12};
static const uint8_t ICM20948_ACCEL_WOM_THR=       {0x13};
static const uint8_t ICM20948_ACCEL_CONFIG=        {0x14};
static const uint8_t ICM20948_ACCEL_CONFIG_2=      {0x15};
static const uint8_t ICM20948_FSYNC_CONFIG=        {0x52};
static const uint8_t ICM20948_TEMP_CONFIG=         {0x53};
static const uint8_t ICM20948_MOD_CTRL_USR=        {0x54};
/* Registers ICM20948 USER BANK 3*/
static const uint8_t ICM20948_I2C_MST_ODR_CFG=     {0x00};
static const uint8_t ICM20948_I2C_MST_CTRL=        {0x01};
static const uint8_t ICM20948_I2C_MST_DELAY_CTRL=  {0x02};
static const uint8_t ICM20948_I2C_SLV0_ADDR=       {0x03};
static const uint8_t ICM20948_I2C_SLV0_REG=        {0x04};
static const uint8_t ICM20948_I2C_SLV0_CTRL=       {0x05};
static const uint8_t ICM20948_I2C_SLV0_DO=         {0x06};
/* Registers ICM20948 ALL BANKS */
static const uint8_t ICM20948_REG_BANK_SEL=        {0x7F};

/* Registers AK09916 */
static const uint8_t AK09916_WIA_1=    {0x00}; // Who I am, Company ID
static const uint8_t AK09916_WIA_2=    {0x01}; // Who I am, Device ID
static const uint8_t AK09916_STATUS_1= {0x10};
static const uint8_t AK09916_HXL=      {0x11};
static const uint8_t AK09916_HXH=      {0x12};
static const uint8_t AK09916_HYL=      {0x13};
static const uint8_t AK09916_HYH=      {0x14};
static const uint8_t AK09916_HZL=      {0x15};
static const uint8_t AK09916_HZH=      {0x16};
static const uint8_t AK09916_STATUS_2= {0x18};
static const uint8_t AK09916_CNTL_2=   {0x31};
static const uint8_t AK09916_CNTL_3=   {0x32};

/* Register Bits */
static const uint8_t ICM20948_RESET=              {0x80};
static const uint8_t ICM20948_I2C_MST_EN=         {0x20};
static const uint8_t ICM20948_SLEEP=              {0x40};
static const uint8_t ICM20948_LP_EN=              {0x20};
static const uint8_t ICM20948_BYPASS_EN =         {0x02};
static const uint8_t ICM20948_GYR_EN=             {0x07};
static const uint8_t ICM20948_ACC_EN=             {0x38};
static const uint8_t ICM20948_FIFO_EN =           {0x40};
static const uint8_t ICM20948_INT1_ACTL =         {0x80};
static const uint8_t ICM20948_INT_1_LATCH_EN =    {0x20};
static const uint8_t ICM20948_ACTL_FSYNC=         {0x08};
static const uint8_t ICM20948_INT_ANYRD_2CLEAR=   {0x10};
static const uint8_t ICM20948_FSYNC_INT_MODE_EN=  {0x06};
static const uint8_t AK09916_16_BIT=              {0x10};
static const uint8_t AK09916_OVF=                 {0x08};
static const uint8_t AK09916_READ=                {0x80};

/* Others */
static const uint16_t AK09916_WHO_AM_I_1=      {0x4809};
static const uint16_t AK09916_WHO_AM_I_2=      {0x0948};
static const uint8_t ICM20948_WHO_AM_I_CONTENT={0xEA};
static const float ICM20948_ROOM_TEMP_OFFSET=  {0.0};
static const float ICM20948_T_SENSITIVITY=     {333.87};
static const float AK09916_MAG_LSB=            {0.1495};

typedef struct {
	/* Reading values */
	int16_t raw_Acc_X;	int8_t raw_SelfT_Acc_X;
	int16_t raw_Acc_Y;	int8_t raw_SelfT_Acc_Y;
	int16_t raw_Acc_Z;	int8_t raw_SelfT_Acc_Z;
	int16_t raw_Gyro_X;	int8_t raw_SelfT_Gyro_X;
	int16_t raw_Gyro_Y;	int8_t raw_SelfT_Gyro_Y;
	int16_t raw_Gyro_Z;	int8_t raw_SelfT_Gyro_Z;
	int16_t raw_Mag_X;
	int16_t raw_Mag_Y;
	int16_t raw_Mag_Z;
	int16_t raw_Temp;

	/* Calculated Values */
	float Acc_Abs;
	float Acc_X;		float Acc_X_0;		float Acc_X_f;		float acc_offset_X;
	float Acc_Y;		float Acc_Y_0;		float Acc_Y_f;		float acc_offset_Y;
	float Acc_Z;		float Acc_Z_0;		float Acc_Z_f;		float acc_offset_Z;
	float acc_sens;
	float acc_lim;

	float Gyro_X;		float Gyro_X_0;		float Gyro_X_f;		float gyro_offset_X;
	float Gyro_Y;		float Gyro_Y_0;		float Gyro_Y_f;		float gyro_offset_Y;
	float Gyro_Z;		float Gyro_Z_0;		float Gyro_Z_f;		float gyro_offset_Z;
	float gyro_sens;

	float Mag_Abs;
	float Mag_X;		float Mag_X_0;		float Mag_X_c;		float Mag_X_f;		float Mag_X_N;		float mag_offset_X;
	float Mag_Y;		float Mag_Y_0;		float Mag_Y_c;		float Mag_Y_f;		float Mag_Y_N;		float mag_offset_Y;
	float Mag_Z;		float Mag_Z_0;		float Mag_Z_c;		float Mag_Z_f;		float Mag_Z_N;		float mag_offset_Z;
	float mag_sens;
	float mag_callb[3][3];
	float mag_lim;

	float Temp;
	float temp_sens;

	/* Euler Angles */
	float Phi;			float Acc_Phi;		float Mag_Phi;
	float Theta;		float Acc_Theta;	float Mag_Theta;
	float Gama;			float Acc_Gama;		float Mag_Gama;

	float Gyro_Phi;		float Gyro_dPhi;	float Gyro_dPhi_f;
	float Gyro_Theta;	float Gyro_dTheta; 	float Gyro_dTheta_f;
	float Gyro_Gama;	float Gyro_dGama;	float Gyro_dGama_f;

	float Acc_Phi_0;	float Gyro_Phi_0;	float Mag_Phi_0;
	float Acc_Theta_0;	float Gyro_Theta_0;	float Mag_Theta_0;
	float Acc_Gama_0;	float Gyro_Gama_0;	float Mag_Gama_0;

    float cosPhi;
    float sinPhi;
    float cosTheta;
    float sinTheta;
    float cosGamma;
    float sinGamma;

	/* Body Frame Infos */
	float Acc_X_NED_0;
	float Acc_Y_NED_0;
	float Acc_Z_NED_0;

	float Acc_X_NED;		float Gyro_X_NED;	float Mag_X_NED;
	float Acc_Y_NED;		float Gyro_Y_NED;	float Mag_Y_NED;
	float Acc_Z_NED;		float Gyro_Z_NED;	float Mag_Z_NED;

	float Vel_X_NED;		float Pos_X_NED;
	float Vel_Y_NED;		float Pos_Y_NED;
	float Vel_Z_NED;		float Pos_Z_NED;

	float Rot_NED[3][3];

	/* Filter Constants */
	float CF_alpha;

	float Acc_DLPF_A[3];	float Acc_DLPF_B[3];

	float Gyro_DLPF_A[4];	float Gyro_DLPF_B[4];
	float Gyro_DHPF_A[4];	float Gyro_DHPF_B[4];

	float Mag_DLPF_A[3];	float Mag_DLPF_B[3];

	/* Complementary Filter */
	float Phi_CF;
	float Theta_CF;
	float Gama_CF;

	/* Plant Dinamics */
	float Ad[2][2];		float Bd[2];			float Cd[2];			float Gd[2];

	/* State Observer */
	float Ke[2];

	float Phi_SO;		float x_Phi_SO_c[2];		float x_Phi_SO_c_past[2];			float x_Phi_SO_p[2];
	float Theta_SO;		float x_Theta_SO_c[2];		float x_Theta_SO_c_past[2];			float x_Theta_SO_p[2];
	float Gama_SO;		float x_Gama_SO_c[2];		float x_Gama_SO_c_past[2];			float x_Gama_SO_p[2];

	/* Kalman Filter */
	float K_KF_Phi[2];	float P_minus_Phi[2][2];	float P_plus_Phi[2][2];				float R_w_Phi;			float R_v_Phi;
	float K_KF_Theta[2];float P_minus_Theta[2][2];	float P_plus_Theta[2][2];			float R_w_Theta;		float R_v_Theta;
	float K_KF_Gama[2];	float P_minus_Gama[2][2];	float P_plus_Gama[2][2];			float R_w_Gama;			float R_v_Gama;

	float Phi_KF;		float x_Phi_KF_c[2];		float x_Phi_KF_c_past[2];			float x_Phi_KF_p[2];
	float Theta_KF;		float x_Theta_KF_c[2];		float x_Theta_KF_c_past[2];			float x_Theta_KF_p[2];
	float Gama_KF;		float x_Gama_KF_c[2];		float x_Gama_KF_c_past[2];			float x_Gama_KF_p[2];

	/* Housekeeping */
	float Acc_X_p[5];	float Acc_X_f_p[5];	float Acc_Phi_p[5];
	float Acc_Y_p[5];	float Acc_Y_f_p[5];	float Acc_Theta_p[5];
	float Acc_Z_p[5];	float Acc_Z_f_p[5];	float Acc_Gama_p[5];

	float Gyro_X_p[5];	float Gyro_X_f_p[5];float Gyro_Phi_p[5];	float Gyro_dPhi_p[5];	float Gyro_dPhi_f_p[5];
	float Gyro_Y_p[5];	float Gyro_Y_f_p[5];float Gyro_Theta_p[5];	float Gyro_dTheta_p[5];	float Gyro_dTheta_f_p[5];
	float Gyro_Z_p[5];	float Gyro_Z_f_p[5];float Gyro_Gama_p[5];	float Gyro_dGama_p[5];	float Gyro_dGama_f_p[5];

	float Mag_X_p[5];	float Mag_X_f_p[5];
	float Mag_Y_p[5];	float Mag_Y_f_p[5];
	float Mag_Z_p[5];	float Mag_Z_f_p[5];

	float Phi_p[5];		float Phi_bias_p[5];
	float Theta_p[5];	float Theta_bias_p[5];
	float Gama_p[5];	float Gama_bias_p[5];

	/* Quaternions */
	Quaternion *q_MW;
	Quaternion *q_KF;

	float BETA; 		//Madgwick beta
	float Q_VARIANCE;	//Ruido do processo para KF
	float R_VARIANCE;	//Ruido de medição para KF
	float P;		float K;		float S;

	float Phi_q_KF;			float Phi_q_MW;
	float Theta_q_KF;		float Theta_q_MW;
	float Gama_q_KF;		float Gama_q_MW;

	/* Timer Values */
	float prev_Time;
	float curr_Time;
	float T_smpl;

	float aux;

	/* Control */
	uint8_t imu_data[22];
	uint8_t bank;

	/* Configuration */
	int8_t status;
	uint8_t PWMT1_IC;
	int8_t Gyro_F_Choice_B;
	int8_t Acc_F_Choice_B;
	int8_t Gyro_F;
	int8_t Acc_F;
	accRange acc_range;
	gyroRange gyro_range;

}IMU;



/* IMU Prototypes */
void imu_begin(SPI_HandleTypeDef spi, GPIO_TypeDef* CS_IMU_, uint16_t CS_IMU_Pin_, IMU* imu);
void imu_config(IMU *imu);
void imu_check_status(IMU* imu);
void imu_reconect_handler(uint8_t trials, IMU*imu);
void imu_bank(uint8_t bank, IMU* imu);
void imu_config_range(accRange acc_rg, gyroRange gyro_rg, IMU* imu);
void imu_config_DLPF(IMU *imu, int8_t G_DLPF_CFG, int8_t A_DLPF_CFG);

void imu_get_data(IMU *imu);
void imu_get_NED_info(IMU *imu);
void imu_get_quaternion_madgwick(IMU *imu);
void imu_get_quaternion_KF(IMU *imu);
void imu_filter_data(IMU *imu);

void imu_get_euler_angles(IMU *imu);
void imu_get_initial_conditions(IMU *imu, int smpls);
void imu_get_samples(IMU *imu, int time, int smp_p_sec, char* arq_name);

void imu_print_data(IMU *imu);
void imu_print_filtered_data(IMU* imu);
void imu_print_raw_data(IMU *imu);
void imu_print_att_data(IMU *imu);
void imu_print_att_data2(IMU *imu);
void imu_print_BF_info(IMU *imu);
void imu_print_data_matlab_raw_and_filtered(IMU* imu);
void imu_print_data_matlab_attitude(IMU* imu);
void imu_print_data_matlab_euler_n_pos(IMU* imu);
void imu_print_data_matlab(IMU* imu);

void imu_Read_Reg(uint8_t reg, uint8_t *data, uint8_t len);
void imu_Write_Reg(uint8_t reg, uint8_t data);

/* MAG Prototypes */
void mag_begin(IMU* imu);
void mag_Read_Reg(uint8_t reg, uint8_t len, IMU *imu);
void mag_Write_Reg(uint8_t reg, uint8_t data, IMU *imu);


#endif /* INC_IMU_H_ */


