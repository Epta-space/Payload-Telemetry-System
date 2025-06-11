/*
 * IMU.c
 *
 *  Created on: Jun 28, 2024
 *      Author: diler
 */

#include "main.h"
#include "IMU.h"

SPI_HandleTypeDef imuspi;
GPIO_TypeDef* CS_IMU_GPIO;
uint16_t CS_IMU_Pin;

/************************IMU Functions************************/

/*Configuration Functions */
void imu_begin(SPI_HandleTypeDef spi, GPIO_TypeDef* CS_IMU_, uint16_t CS_IMU_Pin_, IMU* imu)
{
	imuspi = spi;
	CS_IMU_GPIO = CS_IMU_;
	CS_IMU_Pin = CS_IMU_Pin_;

	uint8_t val=0;

	HAL_Delay(100);

	imu_config(imu);

	imu_Read_Reg(ICM20948_PWR_MGMT_1, &val, 1);
	imu->PWMT1_IC=val;

	/* Variaveis usadas */

	imu->Acc_F_Choice_B=0b0;			//Sem uso de filtros digitais do IMU para o acelerômetro
	imu->Gyro_F_Choice_B=0b0;			//Sem uso de filtros digitais do IMU para o giroscópio
										//								|Medidas feitas
	imu->acc_offset_X=0;				//Seta Bias do acelerômetro em X|
	imu->acc_offset_Y=0;				//Seta Bias do acelerômetro em Y|
	imu->acc_offset_Z=0;				//Sera Bias do acelerômetro em Z|

	imu->gyro_offset_X=0;				//Seta Bias do giroscópio em X	|-0.7150|-0.7607|-0.7429|-0.6969
	imu->gyro_offset_Y=0;				//Seta Bias do giroscópio em Y	|-0.1851|-0.1803|-0.1709|-0.0781
	imu->gyro_offset_Z=0;				//Sera Bias do giroscópio em Z	|-0.0512|+0.0225|+0.0701|+0.0347

	imu->mag_offset_X=0;				//Seta Bias do magnetômetro em X|
	imu->mag_offset_Y=0;				//Seta Bias do magnetômetro em Y|
	imu->mag_offset_Z=0;				//Sera Bias do magnetômetro em Z|

//	imu->mag_offset_X=-38.7842;			//Seta Bias do magnetômetro em X|
//	imu->mag_offset_Y=59.1098;			//Seta Bias do magnetômetro em Y|
//	imu->mag_offset_Z=-14.3299;			//Sera Bias do magnetômetro em Z|

	imu->mag_offset_X=-30.7535;				//Seta Bias do magnetômetro em X|
	imu->mag_offset_Y=56.3195;				//Seta Bias do magnetômetro em Y|
	imu->mag_offset_Z=-16.0127;				//Sera Bias do magnetômetro em Z|

	//Seta matriz de calibração do magnetômetro
	imu->mag_callb[0][0]=1;				imu->mag_callb[0][1]=0;				imu->mag_callb[0][2]=0;
	imu->mag_callb[1][0]=0;				imu->mag_callb[1][1]=1;				imu->mag_callb[1][2]=0;
	imu->mag_callb[2][0]=0;				imu->mag_callb[2][1]=0;				imu->mag_callb[2][2]=1;

	imu->mag_callb[0][0]=0.9161;		imu->mag_callb[0][1]=0;				imu->mag_callb[0][2]=0;
	imu->mag_callb[1][0]=0;				imu->mag_callb[1][1]=1.3681;		imu->mag_callb[1][2]=0;
	imu->mag_callb[2][0]=0;				imu->mag_callb[2][1]=0;				imu->mag_callb[2][2]=0.7979;

//	imu->mag_callb[0][0]=0.9836;		imu->mag_callb[0][1]=-0.0045;		imu->mag_callb[0][2]=0.0161;
//	imu->mag_callb[1][0]=-0.0045;		imu->mag_callb[1][1]=1.2609;		imu->mag_callb[1][2]=-0.0011;
//	imu->mag_callb[2][0]=0.0161;		imu->mag_callb[2][1]=-0.0011;		imu->mag_callb[2][2]=0.8066;

//	imu->mag_callb[0][0]=0.9101;		imu->mag_callb[0][1]=-0.0497;		imu->mag_callb[0][2]=0.0385;
//	imu->mag_callb[1][0]=-0.0497;		imu->mag_callb[1][1]=1.4475;		imu->mag_callb[1][2]=0.0028;
//	imu->mag_callb[2][0]=0.0385;		imu->mag_callb[2][1]=-0.0028;		imu->mag_callb[2][2]=0.7621;

	imu->Phi	=	0;
	imu->Theta	=	0;
	imu->Gama	=	0;

	imu->Gyro_Phi=0;					//Reseta Atitude
	imu->Gyro_Theta=0;					//Reseta Atitude
	imu->Gyro_Gama=0;					//Reseta Atitude

	imu->Mag_Phi=0;						//Reseta Atitude
	imu->Mag_Theta=0;					//Reseta Atitude
	imu->Mag_Gama=0;						//Reseta Atitude

	imu->Acc_Phi_0=0;		imu->Gyro_Phi_0=0;		imu->Mag_Phi_0=0;
	imu->Acc_Theta_0=0;		imu->Gyro_Theta_0=0;	imu->Mag_Theta_0=0;
	imu->Acc_Gama_0=0;		imu->Gyro_Gama_0=0;		imu->Mag_Gama_0=0;

	imu->Vel_X_NED=0;
	imu->Vel_Y_NED=0;
	imu->Vel_Z_NED=0;

	imu->Pos_X_NED=0;
	imu->Pos_Y_NED=0;
	imu->Pos_Z_NED=0;

	imu->acc_lim=0.1;					//Limiar para computar atitude do magnetômetro em [g] absoluto

	imu->mag_lim=1;					//Limiar para computar atitude do magnetômetro em [g] absoluto

	imu->temp_sens = 32767/(85+40);		//Seta sensibilidade do sensor de temperatura????

	imu->mag_sens = 32767/4912;			//Seta sensibilidade do magnetômetro

	//Constantes de filtros
	imu->T_smpl=5;					    //Periodo de amostragem de 5 ms
	imu->curr_Time=0;					//Reseta o tempo lido no presente
	imu->prev_Time=0;					//Reseta o tempo lido no passado

	//Constantes do filtro passa baixa do acelerômetro
	imu->Acc_DLPF_A[0]=0.02008;				imu->Acc_DLPF_B[0]=1;
	imu->Acc_DLPF_A[1]=0.04017;				imu->Acc_DLPF_B[1]=-1.561;
	imu->Acc_DLPF_A[2]=0.02008;				imu->Acc_DLPF_B[2]=0.6414;

	//Constantes do filtro passa baixa do giroscópio
	imu->Gyro_DLPF_A[0]=0.01643;			imu->Gyro_DLPF_B[0]=1;
	imu->Gyro_DLPF_A[1]=0.02957;			imu->Gyro_DLPF_B[1]=-1.617;
	imu->Gyro_DLPF_A[2]=0.01643;			imu->Gyro_DLPF_B[2]=0.6794;

	//Cosntantes para filtro passa alta do giroscópio
	imu->Gyro_DHPF_A[0]=0.755;				imu->Gyro_DHPF_B[0]=1;
	imu->Gyro_DHPF_A[1]=-1.51;				imu->Gyro_DHPF_B[1]=-1.656;
	imu->Gyro_DHPF_A[2]=0.755;				imu->Gyro_DHPF_B[2]=0.7328;

	//Constantes para filtro passa baixa do magnetômetro
	imu->Mag_DLPF_A[0]=0.2483;			imu->Mag_DLPF_B[0]=1;
	imu->Mag_DLPF_A[1]=0.4967;			imu->Mag_DLPF_B[1]=-0.1842;
	imu->Mag_DLPF_A[2]=0.2483;			imu->Mag_DLPF_B[2]=0.1776;

	//Filtro Complementar
	imu->CF_alpha=0.7;					//Constante para o filtro complementar

	// Dinamica do Sistema
	imu->Ad[0][0]=1;				imu->Ad[0][1]=imu->T_smpl/1000;
	imu->Ad[1][0]=0;				imu->Ad[1][1]=1;

	imu->Bd[0]=imu->T_smpl/1000;	imu->Bd[1]=0;

	imu->Cd[0]=1;					imu->Cd[1]=0;

	imu->Ke[0]=0.59;				imu->Ke[1]=17.4;

	imu->Gd[0]=1;					imu->Gd[1]=1;

	//Observador de Estados
	imu->x_Phi_SO_c[0]=0;			imu->x_Phi_SO_c_past[0]=0;			imu->x_Phi_SO_p[0]=0;
	imu->x_Phi_SO_c[1]=0;			imu->x_Phi_SO_c_past[1]=0;			imu->x_Phi_SO_p[1]=0;

	imu->x_Theta_SO_c[0]=0;			imu->x_Theta_SO_c_past[0]=0;		imu->x_Theta_SO_p[0]=0;
	imu->x_Theta_SO_c[1]=0;			imu->x_Theta_SO_c_past[1]=0;		imu->x_Theta_SO_p[1]=0;

	imu->x_Gama_SO_c[0]=0;			imu->x_Gama_SO_c_past[0]=0;			imu->x_Gama_SO_p[0]=0;
	imu->x_Gama_SO_c[1]=0;			imu->x_Gama_SO_c_past[1]=0;			imu->x_Gama_SO_p[1]=0;

	//Filtro de Kalman
	imu->R_w_Phi = 0.01;
	imu->R_v_Phi = 0.05;

	imu->R_w_Theta = 0.01;
	imu->R_v_Theta = 0.05;

	imu->R_w_Gama = 0.01;
	imu->R_v_Gama = 0.05;

	imu->K_KF_Phi[0] = 0; imu->K_KF_Phi[1] = 0;
	imu->P_minus_Phi[0][0] = 1; imu->P_minus_Phi[0][1] = 0; imu->P_minus_Phi[1][0] = 0; imu->P_minus_Phi[1][1] = 1;
	imu->P_plus_Phi[0][0] = 1; imu->P_plus_Phi[0][1] = 0; imu->P_plus_Phi[1][0] = 0; imu->P_plus_Phi[1][1] = 1;

	imu->K_KF_Theta[0] = 0; imu->K_KF_Theta[1] = 0;
	imu->P_minus_Theta[0][0] = 1; imu->P_minus_Theta[0][1] = 0; imu->P_minus_Theta[1][0] = 0; imu->P_minus_Theta[1][1] = 1;
	imu->P_plus_Theta[0][0] = 1; imu->P_plus_Theta[0][1] = 0; imu->P_plus_Theta[1][0] = 0; imu->P_plus_Theta[1][1] = 1;

	imu->K_KF_Gama[0] = 0; imu->K_KF_Gama[1] = 0;
	imu->P_minus_Gama[0][0] = 1; imu->P_minus_Gama[0][1] = 0; imu->P_minus_Gama[1][0] = 0; imu->P_minus_Gama[1][1] = 1;
	imu->P_plus_Gama[0][0] = 1; imu->P_plus_Gama[0][1] = 0; imu->P_plus_Gama[1][0] = 0; imu->P_plus_Gama[1][1] = 1;

	imu->Phi_KF = 0;
	imu->x_Phi_KF_c[0] = 0; imu->x_Phi_KF_c[1] = 0;
	imu->x_Phi_KF_c_past[0] = 0; imu->x_Phi_KF_c_past[1] = 0;
	imu->x_Phi_KF_p[0] = 0; imu->x_Phi_KF_p[1] = 0;

	imu->Theta_KF = 0;
	imu->x_Theta_KF_c[0] = 0; imu->x_Theta_KF_c[1] = 0;
	imu->x_Theta_KF_c_past[0] = 0; imu->x_Theta_KF_c_past[1] = 0;
	imu->x_Theta_KF_p[0] = 0; imu->x_Theta_KF_p[1] = 0;

	imu->Gama_KF = 0;
	imu->x_Gama_KF_c[0] = 0; imu->x_Gama_KF_c[1] = 0;
	imu->x_Gama_KF_c_past[0] = 0; imu->x_Gama_KF_c_past[1] = 0;
	imu->x_Gama_KF_p[0] = 0; imu->x_Gama_KF_p[1] = 0;

	imu->q_MW = (Quaternion *)malloc(sizeof(Quaternion));
	if (imu->q_MW != NULL) {
	    quaternion_init(imu->q_MW, 1.0f, 0.0f, 0.0f, 0.0f);
	}

	imu->q_KF = (Quaternion *)malloc(sizeof(Quaternion));
	if (imu->q_KF != NULL) {
	    quaternion_init(imu->q_KF, 1.0f, 0.0f, 0.0f, 0.0f);
	}

	imu->Q_VARIANCE = 0.5f;
	imu->R_VARIANCE = 0.1f;

	//Zera Housekeeping

	int i;
	for(i=0;i<5;i++)
	{
		imu->Acc_X_p[i]=0;	imu->Acc_X_f_p[i]=0;	imu-> Acc_Phi_p[i]=0;
		imu->Acc_Y_p[i]=0;	imu->Acc_Y_f_p[i]=0;	imu->Acc_Theta_p[i]=0;
		imu->Acc_Z_p[i]=0;	imu->Acc_Z_f_p[i]=0;	imu->Acc_Gama_p[i]=0;

		imu->Gyro_X_p[i]=0;	imu->Gyro_X_f_p[i]=0;	imu->Gyro_Phi_p[i]=0;
		imu->Gyro_Y_p[i]=0;	imu->Gyro_Y_f_p[i]=0;	imu->Gyro_Theta_p[i]=0;
		imu->Gyro_Z_p[i]=0;	imu->Gyro_Z_f_p[i]=0;	imu->Gyro_Gama_p[i]=0;

		imu->Gyro_dPhi_p[i]=0;			imu->Gyro_dPhi_f_p[i]=0;
		imu->Gyro_dTheta_p[i]=0;		imu->Gyro_dTheta_f_p[i]=0;
		imu->Gyro_dGama_p[i]=0;			imu->Gyro_dGama_f_p[i]=0;

		imu->Mag_X_p[i]=0;	imu->Mag_X_f_p[i]=0;
		imu->Mag_Y_p[i]=0;	imu->Mag_Y_f_p[i]=0;
		imu->Mag_Z_p[i]=0;	imu->Mag_Z_f_p[i]=0;

		imu->Phi_p[i]=0;
		imu->Theta_p[i]=0;
		imu->Gama_p[i]=0;
	}
}

void imu_config(IMU *imu)
{
	uint8_t val=0;

	//Checar Who I am
	val=0;
	imu_Read_Reg(ICM20948_WHO_AM_I, &val, 1);
	HAL_Delay(100);

	if (val!=0xEA)
	{
		imu->status=(int)val;
		return;
	}

	//Wake up, resetar tudo
	val=0;
	val|=(1<<7);
	val|=(1<<6);
	val|=1;
	imu_Write_Reg(ICM20948_PWR_MGMT_1, val);
	HAL_Delay(100);

	//Configura o clock interno
	val=1; //Clock interno
	imu_Write_Reg(ICM20948_PWR_MGMT_1, val); //Reg Power Management 1
	HAL_Delay(100);

	//Troca banco de registradores 0>>2
	val=0;
	val|=(2<<4);
	imu_Write_Reg(ICM20948_REG_BANK_SEL, val);

		//Libera ODR
		val=1;
		imu_Write_Reg(ICM20948_ODR_ALIGN_EN, val);
		HAL_Delay(100);

		//Configura sample rate do giroscópio
		val=0; //Max sample rate
		imu_Write_Reg(ICM20948_GYRO_SMPLRT_DIV, val);

		//Configura escala do giroscópio
//		val=0;
//		val|=(1<<1);
//		val|=(1<<2); //Full scale
//		imu_Write_Reg(ICM20948_GYRO_CONFIG_1, val);

		//Configura sample rate do acelerômetro
		val=0;
		imu_Write_Reg(ICM20948_ACCEL_SMPLRT_DIV_1,val);
		imu_Write_Reg(ICM20948_ACCEL_SMPLRT_DIV_2,val);

		//Configura escala do acelerômetro
//		val=0;
//		val|=(1<<1);
//		val|=(1<<2); //Full scale
//		imu_Write_Reg(ICM20948_ACCEL_CONFIG, val); //Reg Accelerometer Configuration

	//Troca banco de registradores 2>>0
	val=0;
	imu_Write_Reg(ICM20948_REG_BANK_SEL, val);

	//Seta comunicação em SPI only
	imu_Read_Reg(ICM20948_USER_CTRL, &val, 1);
	val|=0x10;
	imu_Write_Reg(ICM20948_USER_CTRL,val);

	//Configura FIFO
	val=0;
	val|=1;
	imu_Write_Reg(ICM20948_FIFO_EN_1, val);
	HAL_Delay(100);

	val=0;
	val|=(1<<4);
	val|=(1<<3);
	val|=(1<<2);
	val|=(1<<1);
	val|=1;
	imu_Write_Reg(ICM20948_FIFO_EN_2, val);
	HAL_Delay(100);

	/* Configura os Ranges */
	imu_config_range(ACC_RANGE_16G,		//Configura range de +-16 g para o acelerômetro
					 GYRO_RANGE_2000DPS,//e de +-20000 deg/s par ao giroscópio
					 	 imu);

	/* Inicia comunicação com magnetômetro */
	mag_begin(imu);
	mag_Read_Reg(AK09916_HXL-1,8+1,imu);


/*      NUNCA MAIS DESCOMENTAR ISSO AQUI        */
//	//Liga os sensores
//	val=0;
//	imu_Write_Reg(ICM20948_PWR_MGMT_2, val);
//	HAL_Delay(100);

	val=7;
	imu->status=val; //IMU OK
}

void imu_check_status(IMU* imu)
{
	uint8_t val=0;
	imu_Read_Reg(ICM20948_WHO_AM_I, &val, 1);

	if (val!=0xEA)
	{
		imu->status=-1;
		return;
	}

	val=0;
	imu_Read_Reg(ICM20948_PWR_MGMT_2, &val, 1);

	if (val!=0)
	{
		imu->status=-2;
		return;
	}

	val=0;
	imu_Read_Reg(ICM20948_PWR_MGMT_1, &val, 1);

	if (val!=imu->PWMT1_IC)
	{
		imu->status=-3;
		return;
	}
}

void imu_reconect_handler(uint8_t trials, IMU*imu)
{
	int i=0;

	imu->curr_Time=HAL_GetTick();
	if(imu->curr_Time-imu->prev_Time > imu->T_smpl/2)
	{
		while(trials>i)
		{
			HAL_SPI_Abort(&imuspi);
			HAL_Delay(10);
			HAL_SPI_Init(&imuspi);
			imu->status=7;
			imu_check_status(imu);
			i++;
			printf("Tentativa de reconexão n° %d falhou\r\n",i);
		}
	imu_config(imu);
	}
}

void imu_bank(uint8_t bank, IMU* imu)
{
	if(bank==0||bank==1||bank==2||bank==3)
	{
		if((bank<<4)!=imu->bank)
		{
			imu->bank=bank;
			imu->bank<<=4;
			imu_Write_Reg(ICM20948_REG_BANK_SEL,imu->bank);
		}
	}
}

void imu_config_range(accRange acc_rg, gyroRange gyro_rg, IMU* imu)
{
	imu_bank(2,imu);

	uint8_t ACCEL_CONFIG_=ICM20948_ACCEL_CONFIG
	,GYRO_CONFIG_=ICM20948_GYRO_CONFIG_1
	,G_FCHOICE=imu->Gyro_F_Choice_B
	,A_FCHOICE=imu->Acc_F_Choice_B;

	if(acc_rg==ACC_RANGE_2G){
		imu_Write_Reg(ACCEL_CONFIG_,acc_rg|A_FCHOICE);
		imu->acc_range=acc_rg;
		imu->acc_sens = 32767/2;
	}else if(acc_rg==ACC_RANGE_4G){
		imu_Write_Reg(ACCEL_CONFIG_,acc_rg|A_FCHOICE);
		imu->acc_range=acc_rg;
		imu->acc_sens = 32767/4;
	}else if(acc_rg==ACC_RANGE_8G){
		imu_Write_Reg(ACCEL_CONFIG_,acc_rg|A_FCHOICE);
		imu->acc_range=acc_rg;
		imu->acc_sens = 32767/8;
	}else if(acc_rg==ACC_RANGE_16G){
		imu_Write_Reg(ACCEL_CONFIG_,acc_rg|A_FCHOICE);
		imu->acc_range=acc_rg;
		imu->acc_sens = 32767/16;
	}else{
		printf("Escala de acelerômetro não encontrada");
	}

	if(gyro_rg==GYRO_RANGE_250DPS){
		imu_Write_Reg(GYRO_CONFIG_,gyro_rg|G_FCHOICE);
		imu->gyro_range=gyro_rg;
		imu->gyro_sens = 32767/250;
	}else if(gyro_rg==GYRO_RANGE_500DPS){
		imu_Write_Reg(GYRO_CONFIG_,gyro_rg|G_FCHOICE);
		imu->gyro_range=gyro_rg;
		imu->gyro_sens = 32767/500;
	}else if(gyro_rg==GYRO_RANGE_1000DPS){
		imu_Write_Reg(GYRO_CONFIG_,gyro_rg|G_FCHOICE);
		imu->gyro_range=gyro_rg;
		imu->gyro_sens = 32767/1000;
	}else if(gyro_rg==GYRO_RANGE_2000DPS){
		imu_Write_Reg(GYRO_CONFIG_,gyro_rg|G_FCHOICE);
		imu->gyro_range=gyro_rg;
		imu->gyro_sens = 32767/2000;
	}else {
		printf("Escala de giroscópio não encontrada");
	}

	imu_bank(0,imu);
}

void imu_config_DLPF(IMU *imu, int8_t G_DLPF_CFG, int8_t A_DLPF_CFG)
{
	uint8_t CONFIGURATION = 26
			,GYRO_CONFIG_=27
			,ACCEL_CONFIG_2_=28;
	if(G_DLPF_CFG==0||
			G_DLPF_CFG==1||
			G_DLPF_CFG==2||
			G_DLPF_CFG==3||
			G_DLPF_CFG==4||
			G_DLPF_CFG==5||
			G_DLPF_CFG==6||
			G_DLPF_CFG==7)
	{
		imu->Gyro_F_Choice_B=0b00;
		imu->Gyro_F=G_DLPF_CFG;

		imu_Write_Reg(GYRO_CONFIG_,(uint8_t)imu->gyro_range|imu->Gyro_F_Choice_B);

		imu_Write_Reg(CONFIGURATION,G_DLPF_CFG);
	}else{
		printf("DLPF do giroscópio não encontrado");
	}

	if(A_DLPF_CFG==0||
			A_DLPF_CFG==1||
			A_DLPF_CFG==2||
			A_DLPF_CFG==3||
			A_DLPF_CFG==4||
			A_DLPF_CFG==5||
			A_DLPF_CFG==6||
			A_DLPF_CFG==7)
	{
		imu->Acc_F_Choice_B=0b0;
		imu->Acc_F=A_DLPF_CFG;

		imu_Write_Reg(ACCEL_CONFIG_2_,(uint8_t)imu->Acc_F);
	}else{
		printf("DLPF do acelerômetro não encontrado");
	}
}

/* Acquisition Functions */
void imu_get_data(IMU *imu)
{
	/********************************************************************/
	/* 					Obtenção de dados brutos						*/
	/********************************************************************/

	//Tempo atual
	imu->curr_Time=(float)HAL_GetTick();

	//Leitura dos registradores
	imu_Read_Reg(ICM20948_ACCEL_OUT, imu->imu_data, sizeof(imu->imu_data));

	//Dados brutos de aceleração
	imu->raw_Acc_X=((int16_t)imu->imu_data[0]<<8) + imu->imu_data[1];
	imu->raw_Acc_Y=((int16_t)imu->imu_data[2]<<8) + imu->imu_data[3];
	imu->raw_Acc_Z=((int16_t)imu->imu_data[4]<<8) + imu->imu_data[5];

	//Dados brutos de vel. angular
	imu->raw_Gyro_X=((int16_t)imu->imu_data[6]<<8) + imu->imu_data[7];
	imu->raw_Gyro_Y=((int16_t)imu->imu_data[8]<<8) + imu->imu_data[9];
	imu->raw_Gyro_Z=((int16_t)imu->imu_data[10]<<8) + imu->imu_data[11];

	//Dados brutos de campo magnético
	imu->raw_Mag_X=((int16_t)imu->imu_data[15+1]<<8) + imu->imu_data[14+1];		//X -> X
	imu->raw_Mag_Y=-(((int16_t)imu->imu_data[17+1]<<8) + imu->imu_data[16+1]);	//Y -> -Y
	imu->raw_Mag_Z=-(((int16_t)imu->imu_data[19+1]<<8) + imu->imu_data[18+1]);	//Z -> -Z

	//Dados brutos de temperatura
	imu->raw_Temp=((int16_t)imu->imu_data[12]<<8) + imu->imu_data[13];

	/********************************************************************/
	/* 					Obtenção de dados convertidos					*/
	/********************************************************************/

	// Aceleração em [g]
	imu->Acc_X=(imu->raw_Acc_X/imu->acc_sens)-imu->acc_offset_X;
	imu->Acc_Y=(imu->raw_Acc_Y/imu->acc_sens)-imu->acc_offset_Y;
	imu->Acc_Z=(imu->raw_Acc_Z/imu->acc_sens)-imu->acc_offset_Z;

	// Vel angular em [deg/s]
	imu->Gyro_X=(imu->raw_Gyro_X/imu->gyro_sens)-imu->gyro_offset_X;
	imu->Gyro_Y=(imu->raw_Gyro_Y/imu->gyro_sens)-imu->gyro_offset_Y;
	imu->Gyro_Z=(imu->raw_Gyro_Z/imu->gyro_sens)-imu->gyro_offset_Z;

	// Campo magnético em [uT]
	imu->Mag_X_c=(imu->raw_Mag_X/imu->mag_sens)-imu->mag_offset_X;
	imu->Mag_Y_c=(imu->raw_Mag_Y/imu->mag_sens)-imu->mag_offset_Y;
	imu->Mag_Z_c=(imu->raw_Mag_Z/imu->mag_sens)-imu->mag_offset_Z;

	// Correção do campo magnético
	imu->Mag_X = imu->Mag_X_c*imu->mag_callb[0][0] + imu->Mag_Y_c*imu->mag_callb[0][1] + imu->Mag_Z_c*imu->mag_callb[0][2];
	imu->Mag_Y = imu->Mag_X_c*imu->mag_callb[1][0] + imu->Mag_Y_c*imu->mag_callb[1][1] + imu->Mag_Z_c*imu->mag_callb[1][2];
	imu->Mag_Z = imu->Mag_X_c*imu->mag_callb[2][0] + imu->Mag_Y_c*imu->mag_callb[2][1] + imu->Mag_Z_c*imu->mag_callb[2][2];

	// Temp em [C] (não está certo)
	imu->Temp=imu->raw_Temp/imu->temp_sens-40;

	/********************************************************************/
	/* 					Mudando sistema de coordenadas					*/
	/********************************************************************/
//	imu->aux=imu->Acc_X;
//	imu->Acc_X=-imu->Acc_Z;
//	imu->Acc_Y=-imu->Acc_Y;
//	imu->Acc_Z=imu->aux;
//
//	imu->aux=imu->Gyro_X;
//	imu->Gyro_X=-imu->Gyro_Z;
//	imu->Gyro_Y=-imu->Gyro_Y;
//	imu->Gyro_Z=imu->aux;
//
//	imu->aux=imu->Mag_X;
//	imu->Mag_X=-imu->Mag_Z;
//	imu->Mag_Y=-imu->Mag_Y;
//	imu->Mag_Z=imu->aux;

	//Abaixo só é realizado caso esteja de acordo com o período de amostragem
	if(imu->curr_Time-imu->prev_Time < imu->T_smpl)
	{
		return;
	}

	imu_filter_data(imu);

	imu_get_euler_angles(imu);

	imu_get_quaternion_KF(imu);

	imu_get_quaternion_madgwick(imu);

//	imu->Phi	=	imu->Acc_Phi;
//	imu->Theta	=	imu->Acc_Theta;
//	imu->Gama	=	imu->Acc_Gama;

//	imu->Phi	=	0;
//	imu->Theta	=	0;
//	imu->Gama	=	0;

//	imu->Phi	=	imu->Phi_CF;
//	imu->Theta	=	imu->Theta_CF;
//	imu->Gama	=	imu->Gama_CF;

	imu->Phi	=	imu->Phi_SO;
	imu->Theta	=	imu->Theta_SO;
	imu->Gama	=	imu->Gama_SO;

//	imu->Phi	=	imu->Phi_KF;
//	imu->Theta	=	imu->Theta_KF;
//	imu->Gama	=	imu->Gama_KF;

//	quaternionToEuler(imu->q_MW, &imu->Phi, &imu->Theta, &imu->Gama);
//
//	quaternionToEuler(imu->q_KF, &imu->Phi, &imu->Theta, &imu->Gama);

	imu_get_NED_info(imu);


	/********************************************************************/
	/* 					Troca de período de amostragem					*/
	/********************************************************************/
	int i;
	for(i=4;i!=0;i--)
	{
		imu->Acc_X_p[i]=imu->Acc_X_p[i-1];		imu->Acc_X_f_p[i]=imu->Acc_X_f_p[i-1];		imu-> Acc_Phi_p[i]=imu-> Acc_Phi_p[i-1];
		imu->Acc_Y_p[i]=imu->Acc_Y_p[i-1];		imu->Acc_Y_f_p[i]=imu->Acc_Y_f_p[i-1];		imu->Acc_Theta_p[i]=imu->Acc_Theta_p[i-1];
		imu->Acc_Z_p[i]=imu->Acc_Z_p[i-1];		imu->Acc_Z_f_p[i]=imu->Acc_Z_f_p[i-1];		imu->Acc_Gama_p[i]=imu->Acc_Gama_p[i-1];

		imu->Gyro_X_p[i]=imu->Gyro_X_p[i-1];	imu->Gyro_X_f_p[i]=imu->Gyro_X_f_p[i-1];	imu->Gyro_Phi_p[i]=imu->Gyro_Phi_p[i-1];
		imu->Gyro_Y_p[i]=imu->Gyro_Y_p[i-1];	imu->Gyro_Y_f_p[i]=imu->Gyro_Y_f_p[i-1];	imu->Gyro_Theta_p[i]=imu->Gyro_Theta_p[i-1];
		imu->Gyro_Z_p[i]=imu->Gyro_Z_p[i-1];	imu->Gyro_Z_f_p[i]=imu->Gyro_Z_f_p[i-1];	imu->Gyro_Gama_p[i]=imu->Gyro_Gama_p[i-1];

		imu->Gyro_dPhi_p[i]=imu->Gyro_dPhi_p[i-1];			imu->Gyro_dPhi_f_p[i]=imu->Gyro_dPhi_f_p[i-1];
		imu->Gyro_dTheta_p[i]=imu->Gyro_dTheta_p[i-1];		imu->Gyro_dTheta_f_p[i]=imu->Gyro_dTheta_f_p[i-1];
		imu->Gyro_dGama_p[i]=imu->Gyro_dGama_p[i-1];		imu->Gyro_dGama_f_p[i]=imu->Gyro_dGama_f_p[i-1];

		imu->Phi_p[i]=imu->Phi_p[i-1];
		imu->Theta_p[i]=imu->Theta_p[i-1];
		imu->Gama_p[i]=imu->Gama_p[i-1];
	}

	i=0;
	imu->Acc_X_p[i]=imu->Acc_X;		imu->Acc_X_f_p[i]=imu->Acc_X_f;		imu-> Acc_Phi_p[i]=imu-> Acc_Phi;
	imu->Acc_Y_p[i]=imu->Acc_Y;		imu->Acc_Y_f_p[i]=imu->Acc_Y_f;		imu->Acc_Theta_p[i]=imu->Acc_Theta;
	imu->Acc_Z_p[i]=imu->Acc_Z;		imu->Acc_Z_f_p[i]=imu->Acc_Z_f;		imu->Acc_Gama_p[i]=imu->Acc_Gama;

	imu->Gyro_X_p[i]=imu->Gyro_X;	imu->Gyro_X_f_p[i]=imu->Gyro_X_f;	imu->Gyro_Phi_p[i]=imu->Gyro_Phi;
	imu->Gyro_Y_p[i]=imu->Gyro_Y;	imu->Gyro_Y_f_p[i]=imu->Gyro_Y_f;	imu->Gyro_Theta_p[i]=imu->Gyro_Theta;
	imu->Gyro_Z_p[i]=imu->Gyro_Z;	imu->Gyro_Z_f_p[i]=imu->Gyro_Z_f;	imu->Gyro_Gama_p[i]=imu->Gyro_Gama;

	imu->Gyro_dPhi_p[i]=imu->Gyro_dPhi;			imu->Gyro_dPhi_f_p[i]=imu->Gyro_dPhi_f;
	imu->Gyro_dTheta_p[i]=imu->Gyro_dTheta;		imu->Gyro_dTheta_f_p[i]=imu->Gyro_dTheta_f;
	imu->Gyro_dGama_p[i]=imu->Gyro_dGama;		imu->Gyro_dGama_f_p[i]=imu->Gyro_dGama_f;

	imu->Phi_p[i]=imu->Phi;
	imu->Theta_p[i]=imu->Theta;
	imu->Gama_p[i]=imu->Gama;

	imu->prev_Time=imu->curr_Time;
}

void imu_get_NED_info(IMU *imu)
{
	/********************************************************************/
	/* 					Cálculo de posição, vel e acc					*/
	/********************************************************************/

    imu->cosPhi = cos(imu->Phi);
    imu->sinPhi = sin(imu->Phi);
    imu->cosTheta = cos(imu->Theta);
    imu->sinTheta = sin(imu->Theta);
    imu->cosGamma = cos(imu->Gama);
    imu->sinGamma = sin(imu->Gama);

    // Rot_NED GPT
    imu->Rot_NED[0][0] = imu->cosGamma * imu->cosTheta;
    imu->Rot_NED[0][1] = -(imu->cosGamma * imu->sinTheta * imu->sinPhi - imu->sinGamma * imu->cosPhi);	//Não é negativo pelas ref. bibl. mas funciona apenas assim
    imu->Rot_NED[0][2] = -(imu->cosGamma * imu->sinTheta * imu->cosPhi + imu->sinGamma * imu->sinPhi);	//Não é negativo pelas ref. bibl. mas funciona apenas assim

    imu->Rot_NED[1][0] = imu->sinGamma * imu->cosTheta;
    imu->Rot_NED[1][1] = -(imu->sinGamma * imu->sinTheta * imu->sinPhi + imu->cosGamma * imu->cosPhi);	//Não é negativo pelas ref. bibl. mas funciona apenas assim
    imu->Rot_NED[1][2] = -(imu->sinGamma * imu->sinTheta * imu->cosPhi - imu->cosGamma * imu->sinPhi);

    imu->Rot_NED[2][0] = imu->sinTheta;																	//Não é positivo pelas ref. bibl. mas funciona apenas assi
    imu->Rot_NED[2][1] = imu->cosTheta * imu->sinPhi;
    imu->Rot_NED[2][2] = imu->cosTheta * imu->cosPhi;

    // Rot_NED SteppeSchool / Rot_NED^⁻1 GPT
//	imu->Rot_NED[0][0] = imu->cosGamma * imu->cosTheta;
//	imu->Rot_NED[0][1] = imu->sinGamma * imu->cosTheta;
//	imu->Rot_NED[0][2] = -imu->sinTheta;
//
//	imu->Rot_NED[1][0] = imu->cosGamma * imu->sinTheta * imu->sinPhi - imu->sinGamma * imu->cosPhi;
//	imu->Rot_NED[1][1] = imu->sinGamma * imu->sinTheta * imu->sinPhi + imu->cosGamma * imu->cosPhi;
//	imu->Rot_NED[1][2] = imu->cosTheta * imu->sinPhi;
//
//	imu->Rot_NED[2][0] = imu->cosGamma * imu->sinTheta * imu->cosPhi + imu->sinGamma * imu->sinPhi;
//	imu->Rot_NED[2][1] = imu->sinGamma * imu->sinTheta * imu->cosPhi - imu->cosGamma * imu->sinPhi;
//	imu->Rot_NED[2][2] = imu->cosTheta * imu->cosPhi;

    imu->Acc_X_NED = (imu->Rot_NED[0][0] * imu->Acc_X_f +
					imu->Rot_NED[0][1] * imu->Acc_Y_f +
					imu->Rot_NED[0][2] * imu->Acc_Z_f )
						*9.81 - imu->Acc_X_NED_0;

	imu->Acc_Y_NED = (imu->Rot_NED[1][0] * imu->Acc_X_f +
					imu->Rot_NED[1][1] * imu->Acc_Y_f +
					imu->Rot_NED[1][2] * imu->Acc_Z_f )
						*9.81 - imu->Acc_Y_NED_0;

	imu->Acc_Z_NED = (imu->Rot_NED[2][0] * imu->Acc_X_f +
					imu->Rot_NED[2][1] * imu->Acc_Y_f +
					imu->Rot_NED[2][2] * imu->Acc_Z_f )
						*9.81 - imu->Acc_Z_NED_0;

	if(imu->Acc_X_NED < (imu->acc_lim*9.81) && imu->Acc_X_NED > (-imu->acc_lim*9.81))
	{
		imu->Acc_X_NED=0.0;
	}
	if(imu->Acc_Y_NED < (imu->acc_lim*9.81) && imu->Acc_Y_NED > (-imu->acc_lim*9.81))
	{
		imu->Acc_Y_NED=0.0;
	}
	if(imu->Acc_Z_NED < (imu->acc_lim*9.81) && imu->Acc_Z_NED > (-imu->acc_lim*9.81))
	{
		imu->Acc_Z_NED=0.0;
	}

	imu->Mag_X_NED = imu->Rot_NED[0][0] * imu->Mag_X_f +
					imu->Rot_NED[0][1] * imu->Mag_Y_f +
					imu->Rot_NED[0][2] * imu->Mag_Z_f;

	imu->Mag_Y_NED = imu->Rot_NED[1][0] * imu->Mag_X_f +
					imu->Rot_NED[1][1] * imu->Mag_Y_f +
					imu->Rot_NED[1][2] * imu->Mag_Z_f;

	imu->Mag_Z_NED = imu->Rot_NED[2][0] * imu->Mag_X_f +
					imu->Rot_NED[2][1] * imu->Mag_Y_f +
					imu->Rot_NED[2][2] * imu->Mag_Z_f;

	imu->Vel_X_NED = imu->Vel_X_NED + imu->Acc_X_NED*(imu->curr_Time-imu->prev_Time)/1000;
	imu->Vel_Y_NED = imu->Vel_Y_NED + imu->Acc_Y_NED*(imu->curr_Time-imu->prev_Time)/1000;
	imu->Vel_Z_NED = imu->Vel_Z_NED + imu->Acc_Z_NED*(imu->curr_Time-imu->prev_Time)/1000;

	imu->Pos_X_NED+=imu->Vel_X_NED*(imu->curr_Time-imu->prev_Time)/1000;
	imu->Pos_Y_NED+=imu->Vel_Y_NED*(imu->curr_Time-imu->prev_Time)/1000;
	imu->Pos_Z_NED+=imu->Vel_Z_NED*(imu->curr_Time-imu->prev_Time)/1000;

	if(imu->Vel_X_NED < (imu->acc_lim*9.81) && imu->Vel_X_NED > (-imu->acc_lim*9.81))
	{
		imu->Vel_X_NED=0.0;
	}
	if(imu->Vel_Y_NED < (imu->acc_lim*9.81) && imu->Vel_Y_NED > (-imu->acc_lim*9.81))
	{
		imu->Vel_Y_NED=0.0;
	}
	if(imu->Vel_Z_NED < (imu->acc_lim*9.81) && imu->Vel_Z_NED > (-imu->acc_lim*9.81))
	{
		imu->Vel_Z_NED=0.0;
	}

}

void imu_get_quaternion_KF(IMU *imu)
{
    float ax=imu->Acc_X_f, ay=imu->Acc_Y_f, az=imu->Acc_Z_f;
    float gx=imu->Gyro_X_f, gy=imu->Gyro_Y_f, gz=imu->Gyro_Z_f;
    float mx=imu->Mag_X_f, my=imu->Mag_Y_f, mz=imu->Mag_Z_f;
    float dt=(imu->curr_Time-imu->prev_Time)/1000;

    // Normaliza valores
    normalizeVector(&ax, &ay, &az);
    normalizeVector(&mx, &my, &mz);

    // Converte taxas do giroscópio para radianos por segundo
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    /* Etapa de Predição */

    // Atualiza quaternion com integração do giroscópio
    Quaternion qDot = {
        -imu->q_KF->x * gx - imu->q_KF->y * gy - imu->q_KF->z * gz,
		imu->q_KF->w * gx + imu->q_KF->y * gz - imu->q_KF->z * gy,
		imu->q_KF->w * gy - imu->q_KF->x * gz + imu->q_KF->z * gx,
		imu->q_KF->w * gz + imu->q_KF->x * gy - imu->q_KF->y * gx
    };

    // Atualiza o quaternion
    imu->q_KF->w += qDot.w;
    imu->q_KF->x += qDot.x;
    imu->q_KF->y += qDot.y;
    imu->q_KF->z += qDot.z;

    // Normaliza quaternion
    normalizeQuaternion(imu->q_KF);

    // Atualiza a incerteza da previsão (P)
    imu->P += imu->Q_VARIANCE;

    /* Etapa de Correção */

    // Estima a orientação do acelerômetro
    Quaternion qAcc = {
        sqrt(0.5f * (1.0f + ax)),
        sqrt(0.5f * (1.0f - ax)),
        sqrt(0.5f * (1.0f + ay)),
        sqrt(0.5f * (1.0f - ay))
    };
    normalizeQuaternion(&qAcc);

    // Cálculo do erro entre estado previsto e medido
    float errX = imu->q_KF->x - qAcc.x;
    float errY = imu->q_KF->y - qAcc.y;
    float errZ = imu->q_KF->z - qAcc.z;

    // Covariância da medição
    imu->S = imu->P + imu->R_VARIANCE;

    // Ganho de Kalman
    imu->K = imu->P / imu->S;

    // Atualiza quaternion corrigindo erro
    imu->q_KF->x -= imu->K * errX;
    imu->q_KF->y -= imu->K * errY;
    imu->q_KF->z -= imu->K * errZ;
    imu->q_KF->w = sqrt(1.0f - imu->q_KF->x * imu->q_KF->x - imu->q_KF->y * imu->q_KF->y - imu->q_KF->z * imu->q_KF->z); // Recalcula w

    // Normaliza quaternion
    normalizeQuaternion(imu->q_KF);

    // Atualiza a covariância
    imu->P *= (1.0f - imu->K);

}

void imu_get_quaternion_madgwick(IMU *imu)
{

	/********************************************************************/
	/* 					Cálculo de quaternion por Madgwick				*/
	/********************************************************************/
    float ax=imu->Acc_X_f, ay=imu->Acc_Y_f, az=imu->Acc_Z_f;
    float gx=imu->Gyro_X_f, gy=imu->Gyro_Y_f, gz=imu->Gyro_Z_f;
    float mx=imu->Mag_X_f, my=imu->Mag_Y_f, mz=imu->Mag_Z_f;
    float dt=(imu->curr_Time-imu->prev_Time)/1000;

    float q0 = imu->q_MW->w, q1 = imu->q_MW->x, q2 = imu->q_MW->y, q3 = imu->q_MW->z;
    float hx, hy, _2bx, _2bz;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;

    // Converte taxas do giroscópio para radianos por segundo
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float BETA = 0.9;

    // Normaliza os vetores do acelerômetro e magnetômetro
    normalizeVector(&ax, &ay, &az);
    normalizeVector(&mx, &my, &mz);

    // Pré-cálculos para evitar operações repetidas
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Computa o vetor de referência do campo magnético (no referencial do sensor)
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    _2bx = sqrtf(hx * hx + hy * hy);
    // Aqui _2bz incorpora o componente vertical do campo magnético
    _2bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

    // Agora computa o passo corretivo (gradiente) para a fusão dos sensores
    // Note que os termos envolvendo _2bz são usados para levar em conta a correção do yaw
    s0 = -_2q2 * (2.0f * q1q3 - _2q0 * q2 - ax)
         + _2q1 * (2.0f * q0q1 + _2q2 * q3 - ay)
         - _2bz * q2 * (_2bx * (0.5f - q3q3 - q0q0) + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q0q1) + _2bz * (q0q3 + q1q2) - my)
         + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s1 = _2q3 * (2.0f * q1q3 - _2q0 * q2 - ax)
         + _2q0 * (2.0f * q0q1 + _2q2 * q3 - ay)
         - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
         + _2bz * q3 * (_2bx * (0.5f - q3q3 - q0q0) + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q2 + _2bz * q0) * (_2bx * (q2q3 - q0q1) + _2bz * (q0q3 + q1q2) - my)
         + (_2bx * q3 - 4.0f * _2bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s2 = -_2q0 * (2.0f * q1q3 - _2q0 * q2 - ax)
         + _2q3 * (2.0f * q0q1 + _2q2 * q3 - ay)
         - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
         + (-4.0f * _2bx * q2 - _2bz * q0) * (_2bx * (0.5f - q3q3 - q0q0) + _2bz * (q1q3 - q0q2) - mx)
         + (_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q0q1) + _2bz * (q0q3 + q1q2) - my)
         + (_2bx * q0 - 4.0f * _2bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    s3 = _2q1 * (2.0f * q1q3 - _2q0 * q2 - ax)
         + _2q2 * (2.0f * q0q1 + _2q2 * q3 - ay)
         + (-4.0f * _2bx * q3 + _2bz * q1) * (_2bx * (0.5f - q3q3 - q0q0) + _2bz * (q1q3 - q0q2) - mx)
         + (-_2bx * q0 + _2bz * q2) * (_2bx * (q2q3 - q0q1) + _2bz * (q0q3 + q1q2) - my)
         + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    // Normaliza o vetor gradiente
    float recipNorm = 1/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if(recipNorm<=0){return;}
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Calcula a taxa de variação do quaternion
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - BETA * s0;
    qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - BETA * s1;
    qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - BETA * s2;
    qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - BETA * s3;

    // Integra para obter o novo quaternion
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normaliza quaternions
    imu->q_MW->w = q0;
    imu->q_MW->x = q1;
    imu->q_MW->y = q2;
    imu->q_MW->z = q3;
    normalizeQuaternion(imu->q_MW);

}


void imu_get_euler_angles(IMU *imu)
{	/********************************************************************/
	/* 							Calculo de Atitude 						*/
	/********************************************************************/

	imu->Acc_Abs=sqrt(pow(imu->Acc_X_f,2)+pow(imu->Acc_Y_f,2)+pow(imu->Acc_Z_f,2));
	imu->Mag_Abs=sqrt(pow(imu->Mag_X,2)+pow(imu->Mag_Y,2)+pow(imu->Mag_Z,2));

	//Atitude a partir do acelerômetro
	if(imu->Acc_Abs<1+imu->acc_lim || imu->Acc_Abs>1-imu->acc_lim)
	{
		if(imu->Acc_Z_f>imu->acc_lim*2 || imu->Acc_Z_f<-imu->acc_lim*2)
		{
			imu->Acc_Phi=atan2(imu->Acc_Y_f,imu->Acc_Z_f);
			imu->Acc_Theta=asin(imu->Acc_X_f/imu->Acc_Abs);
		}
		else
		{
			imu->Acc_Phi=atan2(imu->Acc_Y_f,sqrt(pow(imu->Acc_Z_f,2)+pow(imu->Acc_X_f,2)));
			imu->Acc_Theta=asin(imu->Acc_X_f/imu->Acc_Abs);
		}
		imu->Acc_Gama=0;
	}

	//Atitude a partir do magnetômetro

	imu->Mag_X_N=imu->Mag_X_f/imu->Mag_Abs;
	imu->Mag_Y_N=imu->Mag_Y_f/imu->Mag_Abs;
	imu->Mag_Z_N=imu->Mag_Z_f/imu->Mag_Abs;

	imu->Mag_Phi=0;
	imu->Mag_Theta=0;
	imu->Mag_Gama=-atan2(imu->Mag_Z_N*sin(imu->Phi)-imu->Mag_Y_N*cos(imu->Phi),
					imu->Mag_X_N*cos(imu->Theta)+(imu->Mag_Y_N
					*sin(imu->Phi)+imu->Mag_Z_N*cos(imu->Phi))*sin(imu->Theta)-imu->Mag_Gama_0);

	//Atitude a partir do giroscópio
	imu->Gyro_X_NED	=	imu->Gyro_X_f	+	sin(imu->Phi)*tan(imu->Theta)*imu->Gyro_Y_f	+	cos(imu->Phi)*tan(imu->Theta)*imu->Gyro_Z_f;
	imu->Gyro_Y_NED	=	-cos(imu->Phi)*imu->Gyro_Y_f						+	sin(imu->Phi)*imu->Gyro_Z_f;
	imu->Gyro_Z_NED	=	sin(imu->Phi)/cos(imu->Theta)*imu->Gyro_Y_f		+	cos(imu->Phi)/cos(imu->Theta)*imu->Gyro_Z_f;

	imu->Gyro_dPhi	=	imu->Gyro_X_NED*(imu->curr_Time-imu->prev_Time)/1000		/180*M_PI;
	imu->Gyro_dTheta=	imu->Gyro_Y_NED*(imu->curr_Time-imu->prev_Time)/1000		/180*M_PI;
	imu->Gyro_dGama	=	imu->Gyro_Z_NED*(imu->curr_Time-imu->prev_Time)/1000		/180*M_PI;

	//Filtragem por filtro passa alta

	imu->Gyro_dPhi_f=	(imu->Gyro_dPhi*imu->Gyro_DHPF_A[0] +	imu->Gyro_dPhi_p[0]*imu->Gyro_DHPF_A[1]	+ imu->Gyro_dPhi_p[1]*imu->Gyro_DHPF_A[2]
					    -imu->Gyro_dPhi_f_p[0]*imu->Gyro_DHPF_B[1]	-	imu->Gyro_dPhi_f_p[1]*imu->Gyro_DHPF_B[2])
							/imu->Gyro_DHPF_B[0];

	imu->Gyro_dTheta_f=	(imu->Gyro_dTheta*imu->Gyro_DHPF_A[0] +	imu->Gyro_dTheta_p[0]*imu->Gyro_DHPF_A[1]+imu->Gyro_dTheta_p[1]*imu->Gyro_DHPF_A[2]
					    -imu->Gyro_dTheta_f_p[0]*imu->Gyro_DHPF_B[1]-	imu->Gyro_dTheta_f_p[1]*imu->Gyro_DHPF_B[2])
							/imu->Gyro_DHPF_B[0];

	imu->Gyro_dGama_f=	(imu->Gyro_dGama*imu->Gyro_DHPF_A[0] +	imu->Gyro_dGama_p[0]*imu->Gyro_DHPF_A[1] +imu->Gyro_dGama_p[1]*imu->Gyro_DHPF_A[2]
					    -imu->Gyro_dGama_f_p[0]*imu->Gyro_DHPF_B[1]	-	imu->Gyro_dGama_f_p[1]*imu->Gyro_DHPF_B[2])
							/imu->Gyro_DHPF_B[0];

	imu->Gyro_dPhi_f=imu->Gyro_dPhi;
	imu->Gyro_dTheta_f=imu->Gyro_dTheta;
	imu->Gyro_dGama_f=imu->Gyro_dGama;

//	imu->Gyro_Phi+=imu->Gyro_dPhi_f;
//	imu->Gyro_Theta+=imu->Gyro_dTheta_f;
//	imu->Gyro_Gama+=imu->Gyro_dGama_f;

	imu->Gyro_Phi=imu->Phi_CF			+	imu->Gyro_dPhi_f;
	imu->Gyro_Theta=imu->Theta_CF		+	imu->Gyro_dTheta_f;
	imu->Gyro_Gama=imu->Gyro_Gama		+	imu->Gyro_dGama_f;

	/********************************************************************/
	/* 			Filtro complementar para estimativa de atitude			*/
	/********************************************************************/

	imu->Phi_CF	 =	imu->CF_alpha*imu->Acc_Phi 		+ 	(1-imu->CF_alpha)*imu->Gyro_Phi;
	imu->Theta_CF=	imu->CF_alpha*imu->Acc_Theta 	+ 	(1-imu->CF_alpha)*imu->Gyro_Theta;
	imu->Gama_CF =	imu->CF_alpha*imu->Mag_Gama 	+ 	(1-imu->CF_alpha)*imu->Gyro_Gama;

	/********************************************************************/
	/* 			Observador de estados para estimativa de atitude		*/
	/********************************************************************/

	/* Correção */

	//Em Phi
    imu->x_Phi_SO_c[0] = imu->x_Phi_SO_p[0] + imu->Ke[0]*(imu->Acc_Phi-(imu->Cd[0]*imu->x_Phi_SO_p[0]+imu->Cd[1]*imu->x_Phi_SO_p[1]));
    imu->x_Phi_SO_c[1] = imu->x_Phi_SO_p[1] + imu->Ke[1]*(imu->Acc_Phi-(imu->Cd[0]*imu->x_Phi_SO_p[0]+imu->Cd[1]*imu->x_Phi_SO_p[1]));

    imu->Phi_SO=imu->x_Phi_SO_c[0];

	//Em Theta
    imu->x_Theta_SO_c[0] = imu->x_Theta_SO_p[0] + imu->Ke[0]*(imu->Acc_Theta-(imu->Cd[0]*imu->x_Theta_SO_p[0]+imu->Cd[1]*imu->x_Theta_SO_p[1]));
    imu->x_Theta_SO_c[1] = imu->x_Theta_SO_p[1] + imu->Ke[1]*(imu->Acc_Theta-(imu->Cd[0]*imu->x_Theta_SO_p[0]+imu->Cd[1]*imu->x_Theta_SO_p[1]));

    imu->Theta_SO=imu->x_Theta_SO_c[0];

	//Em Gama
    imu->x_Gama_SO_c[0] = imu->x_Gama_SO_p[0] + imu->Ke[0]*(imu->Mag_Gama-(imu->Cd[0]*imu->x_Gama_SO_p[0]+imu->Cd[1]*imu->x_Gama_SO_p[1]));
    imu->x_Gama_SO_c[1] = imu->x_Gama_SO_p[1] + imu->Ke[1]*(imu->Mag_Gama-(imu->Cd[0]*imu->x_Gama_SO_p[0]+imu->Cd[1]*imu->x_Gama_SO_p[1]));

    imu->Gama_SO=imu->x_Gama_SO_c[0];

	/* Previsão */

	//Em Phi
    imu->x_Phi_SO_p[0] = imu->Ad[0][0]*imu->x_Phi_SO_c_past[0] + imu->Ad[0][1]*imu->x_Phi_SO_c_past[1] + imu->Gyro_dPhi_f;//imu->Bd[0]*imu->Gyro_Phi;
    imu->x_Phi_SO_p[1] = imu->Ad[1][0]*imu->x_Phi_SO_c_past[0] + imu->Ad[1][1]*imu->x_Phi_SO_c_past[1] + imu->Bd[1]*imu->Gyro_Phi;

	//Em Theta
    imu->x_Theta_SO_p[0] = imu->Ad[0][0]*imu->x_Theta_SO_c_past[0] + imu->Ad[0][1]*imu->x_Theta_SO_c_past[1] + imu->Gyro_dTheta_f;//imu->Bd[0]*imu->Gyro_Theta;
    imu->x_Theta_SO_p[1] = imu->Ad[1][0]*imu->x_Theta_SO_c_past[0] + imu->Ad[1][1]*imu->x_Theta_SO_c_past[1] + imu->Bd[1]*imu->Gyro_Theta;

	//Em Gama
    imu->x_Gama_SO_p[0] = imu->Ad[0][0]*imu->x_Gama_SO_c_past[0] + imu->Ad[0][1]*imu->x_Gama_SO_c_past[1] + imu->Gyro_dGama_f;//imu->Bd[0]*imu->Gyro_Gama;
    imu->x_Gama_SO_p[1] = imu->Ad[1][0]*imu->x_Gama_SO_c_past[0] + imu->Ad[1][1]*imu->x_Gama_SO_c_past[1] + imu->Bd[1]*imu->Gyro_Gama;

	/* Observador de Estados */
	imu->x_Phi_SO_c_past[0]=imu->x_Phi_SO_c[0];
	imu->x_Phi_SO_c_past[1]=imu->x_Phi_SO_c[1];

	imu->x_Theta_SO_c_past[0]=imu->x_Theta_SO_c[0];
	imu->x_Theta_SO_c_past[1]=imu->x_Theta_SO_c[1];

	imu->x_Gama_SO_c_past[0]=imu->x_Gama_SO_c[0];
	imu->x_Gama_SO_c_past[1]=imu->x_Gama_SO_c[1];

	/********************************************************************/
	/* 			Filtro de Kalman para estimativa de atitude				*/
	/********************************************************************/

	/* Correção */

	//Em Phi
    imu->K_KF_Phi[0] = (imu->P_minus_Phi[0][0] * imu->Cd[0] + imu->P_minus_Phi[0][1] *imu->Cd[1])/(
    		(imu->Cd[0] * imu->P_minus_Phi[0][0]+imu->Cd[1] * imu->P_minus_Phi[1][0])*imu->Cd[0] +
			(imu->Cd[0] * imu->P_minus_Phi[0][1]+imu->Cd[1] * imu->P_minus_Phi[1][1])*imu->Cd[1] +
			 imu->R_v_Phi );
    imu->K_KF_Phi[1] = (imu->P_minus_Phi[1][0] * imu->Cd[0] + imu->P_minus_Phi[1][1] *imu->Cd[1])/(
    		(imu->Cd[0] * imu->P_minus_Phi[0][0]+imu->Cd[1] * imu->P_minus_Phi[1][0])*imu->Cd[0] +
			(imu->Cd[0] * imu->P_minus_Phi[0][1]+imu->Cd[1] * imu->P_minus_Phi[1][1])*imu->Cd[1] +
			 imu->R_v_Phi );


    imu->x_Phi_KF_c[0] = imu->x_Phi_KF_p[0] + imu->K_KF_Phi[0]*(imu->Acc_Phi-(imu->Cd[0]*imu->x_Phi_KF_p[0]+imu->Cd[1]*imu->x_Phi_KF_p[1]));
    imu->x_Phi_KF_c[1] = imu->x_Phi_KF_p[1] + imu->K_KF_Phi[1]*(imu->Acc_Phi-(imu->Cd[0]*imu->x_Phi_KF_p[0]+imu->Cd[1]*imu->x_Phi_KF_p[1]));

    imu->Phi_KF=imu->x_Phi_KF_c[0];

	//Em Theta
    imu->K_KF_Theta[0] = (imu->P_minus_Theta[0][0] * imu->Cd[0] + imu->P_minus_Theta[0][1] *imu->Cd[1])/(
    		(imu->Cd[0] * imu->P_minus_Theta[0][0]+imu->Cd[1] * imu->P_minus_Theta[1][0])*imu->Cd[0] +
			(imu->Cd[0] * imu->P_minus_Theta[0][1]+imu->Cd[1] * imu->P_minus_Theta[1][1])*imu->Cd[1] +
			 imu->R_v_Theta );
    imu->K_KF_Theta[1] = (imu->P_minus_Theta[1][0] * imu->Cd[0] + imu->P_minus_Theta[1][1] *imu->Cd[1])/(
    		(imu->Cd[0] * imu->P_minus_Theta[0][0]+imu->Cd[1] * imu->P_minus_Theta[1][0])*imu->Cd[0] +
			(imu->Cd[0] * imu->P_minus_Theta[0][1]+imu->Cd[1] * imu->P_minus_Theta[1][1])*imu->Cd[1] +
			 imu->R_v_Theta );

    imu->x_Theta_KF_c[0] = imu->x_Theta_KF_p[0] + imu->K_KF_Theta[0]*(imu->Acc_Theta-(imu->Cd[0]*imu->x_Theta_KF_p[0]+imu->Cd[1]*imu->x_Theta_KF_p[1]));
    imu->x_Theta_KF_c[1] = imu->x_Theta_KF_p[1] + imu->K_KF_Theta[1]*(imu->Acc_Theta-(imu->Cd[0]*imu->x_Theta_KF_p[0]+imu->Cd[1]*imu->x_Theta_KF_p[1]));

    imu->Theta_KF=imu->x_Theta_KF_c[0];

	//Em Gama
    imu->K_KF_Gama[0] = (imu->P_minus_Gama[0][0] * imu->Cd[0] + imu->P_minus_Gama[0][1] *imu->Cd[1])/(
    		(imu->Cd[0] * imu->P_minus_Gama[0][0]+imu->Cd[1] * imu->P_minus_Gama[1][0])*imu->Cd[0] +
			(imu->Cd[0] * imu->P_minus_Gama[0][1]+imu->Cd[1] * imu->P_minus_Gama[1][1])*imu->Cd[1] +
			imu->R_v_Gama );
    imu->K_KF_Gama[1] = (imu->P_minus_Gama[1][0] * imu->Cd[0] + imu->P_minus_Gama[1][1] *imu->Cd[1])/(
    		(imu->Cd[0] * imu->P_minus_Gama[0][0]+imu->Cd[1] * imu->P_minus_Gama[1][0])*imu->Cd[0] +
			(imu->Cd[0] * imu->P_minus_Gama[0][1]+imu->Cd[1] * imu->P_minus_Gama[1][1])*imu->Cd[1] +
			imu->R_v_Gama );

    imu->x_Gama_KF_c[0] = imu->x_Gama_KF_p[0] + imu->K_KF_Gama[0]*(imu->Mag_Gama-(imu->Cd[0]*imu->x_Gama_KF_p[0]+imu->Cd[1]*imu->x_Gama_KF_p[1]));
    imu->x_Gama_KF_c[1] = imu->x_Gama_KF_p[1] + imu->K_KF_Gama[1]*(imu->Mag_Gama-(imu->Cd[0]*imu->x_Gama_KF_p[0]+imu->Cd[1]*imu->x_Gama_KF_p[1]));

    imu->Gama_KF=imu->x_Gama_KF_c[0];

	/* Propagação */

	//Em Phi
    imu->P_plus_Phi[0][0] = (1-imu->K_KF_Phi[0]*imu->Cd[0])*imu->P_minus_Phi[0][0] - imu->K_KF_Phi[0]*imu->Cd[1]*imu->P_minus_Phi[1][0];
    imu->P_plus_Phi[1][0] = -imu->K_KF_Phi[1]*imu->Cd[0]*imu->P_minus_Phi[0][0] + (1-imu->K_KF_Phi[1]*imu->Cd[1])*imu->P_minus_Phi[1][0];
    imu->P_plus_Phi[0][1] = (1-imu->K_KF_Phi[0]*imu->Cd[0])*imu->P_minus_Phi[0][1] - imu->K_KF_Phi[0]*imu->Cd[1]*imu->P_minus_Phi[1][1];
    imu->P_plus_Phi[1][1] = -imu->K_KF_Phi[1]*imu->Cd[0]*imu->P_minus_Phi[0][1] + (1-imu->K_KF_Phi[1]*imu->Cd[1])*imu->P_minus_Phi[1][1];

    imu->P_minus_Phi[0][0] = (imu->Ad[0][0]*imu->P_plus_Phi[0][0] + imu->Ad[0][1]*imu->P_plus_Phi[1][0])*imu->Ad[0][0]
							+(imu->Ad[0][0]*imu->P_plus_Phi[0][1] + imu->Ad[0][1]*imu->P_plus_Phi[1][1])*imu->Ad[0][1]
					        +imu->R_w_Phi*(imu->Gd[0]*imu->Gd[0]);
    imu->P_minus_Phi[1][0] = (imu->Ad[1][0]*imu->P_plus_Phi[0][0] + imu->Ad[1][1]*imu->P_plus_Phi[1][0])*imu->Ad[0][0]
							+(imu->Ad[1][0]*imu->P_plus_Phi[0][1] + imu->Ad[1][1]*imu->P_plus_Phi[1][1])*imu->Ad[0][1]
					        +imu->R_w_Phi*(imu->Gd[1]*imu->Gd[0]);
    imu->P_minus_Phi[0][1] = (imu->Ad[0][0]*imu->P_plus_Phi[0][0] + imu->Ad[0][1]*imu->P_plus_Phi[1][0])*imu->Ad[1][0]
							+(imu->Ad[0][0]*imu->P_plus_Phi[0][1] + imu->Ad[0][1]*imu->P_plus_Phi[1][1])*imu->Ad[1][1]
					        +imu->R_w_Phi*(imu->Gd[0]*imu->Gd[1]);
    imu->P_minus_Phi[1][1] = (imu->Ad[1][0]*imu->P_plus_Phi[0][0] + imu->Ad[1][1]*imu->P_plus_Phi[1][0])*imu->Ad[1][0]
							+(imu->Ad[1][0]*imu->P_plus_Phi[0][1] + imu->Ad[1][1]*imu->P_plus_Phi[1][1])*imu->Ad[1][1]
					        +imu->R_w_Phi*(imu->Gd[1]*imu->Gd[1]);

    imu->x_Phi_KF_p[0] = imu->Ad[0][0]*imu->x_Phi_KF_c_past[0] + imu->Ad[0][1]*imu->x_Phi_KF_c_past[1] + imu->Gyro_Phi;//imu->Bd[0]*imu->Gyro_Phi;
    imu->x_Phi_KF_p[1] = imu->Ad[1][0]*imu->x_Phi_KF_c_past[0] + imu->Ad[1][1]*imu->x_Phi_KF_c_past[1] + imu->Bd[1]*imu->Gyro_Phi;

	//Em Theta
    imu->P_plus_Theta[0][0] = (1-imu->K_KF_Theta[0]*imu->Cd[0])*imu->P_minus_Theta[0][0] - imu->K_KF_Theta[0]*imu->Cd[1]*imu->P_minus_Theta[1][0];
    imu->P_plus_Theta[1][0] = -imu->K_KF_Theta[1]*imu->Cd[0]*imu->P_minus_Theta[0][0] + (1-imu->K_KF_Theta[1]*imu->Cd[1])*imu->P_minus_Theta[1][0];
    imu->P_plus_Theta[0][1] = (1-imu->K_KF_Theta[0]*imu->Cd[0])*imu->P_minus_Theta[0][1] - imu->K_KF_Theta[0]*imu->Cd[1]*imu->P_minus_Theta[1][1];
    imu->P_plus_Theta[1][1] = -imu->K_KF_Theta[1]*imu->Cd[0]*imu->P_minus_Theta[0][1] + (1-imu->K_KF_Theta[1]*imu->Cd[1])*imu->P_minus_Theta[1][1];

    imu->P_minus_Theta[0][0] = (imu->Ad[0][0]*imu->P_plus_Theta[0][0] + imu->Ad[0][1]*imu->P_plus_Theta[1][0])*imu->Ad[0][0]
							  +(imu->Ad[0][0]*imu->P_plus_Theta[0][1] + imu->Ad[0][1]*imu->P_plus_Theta[1][1])*imu->Ad[0][1]
					          +imu->R_w_Theta*(imu->Gd[0]*imu->Gd[0]);
    imu->P_minus_Theta[1][0] = (imu->Ad[1][0]*imu->P_plus_Theta[0][0] + imu->Ad[1][1]*imu->P_plus_Theta[1][0])*imu->Ad[0][0]
							  +(imu->Ad[1][0]*imu->P_plus_Theta[0][1] + imu->Ad[1][1]*imu->P_plus_Theta[1][1])*imu->Ad[0][1]
					          +imu->R_w_Theta*(imu->Gd[1]*imu->Gd[0]);
    imu->P_minus_Theta[0][1] = (imu->Ad[0][0]*imu->P_plus_Theta[0][0] + imu->Ad[0][1]*imu->P_plus_Theta[1][0])*imu->Ad[1][0]
						      +(imu->Ad[0][0]*imu->P_plus_Theta[0][1] + imu->Ad[0][1]*imu->P_plus_Theta[1][1])*imu->Ad[1][1]
					          +imu->R_w_Theta*(imu->Gd[0]*imu->Gd[1]);
    imu->P_minus_Theta[1][1] = (imu->Ad[1][0]*imu->P_plus_Theta[0][0] + imu->Ad[1][1]*imu->P_plus_Theta[1][0])*imu->Ad[1][0]
							  +(imu->Ad[1][0]*imu->P_plus_Theta[0][1] + imu->Ad[1][1]*imu->P_plus_Theta[1][1])*imu->Ad[1][1]
					          +imu->R_w_Theta*(imu->Gd[1]*imu->Gd[1]);

    imu->x_Theta_KF_p[0] = imu->Ad[0][0]*imu->x_Theta_KF_c_past[0] + imu->Ad[0][1]*imu->x_Theta_KF_c_past[1] + imu->Gyro_Theta;//imu->Bd[0]*imu->Gyro_Theta;
    imu->x_Theta_KF_p[1] = imu->Ad[1][0]*imu->x_Theta_KF_c_past[0] + imu->Ad[1][1]*imu->x_Theta_KF_c_past[1] + imu->Bd[1]*imu->Gyro_Theta;

	//Em Gama
    imu->P_plus_Gama[0][0] = (1-imu->K_KF_Gama[0]*imu->Cd[0])*imu->P_minus_Gama[0][0] - imu->K_KF_Gama[0]*imu->Cd[1]*imu->P_minus_Gama[1][0];
    imu->P_plus_Gama[1][0] = -imu->K_KF_Gama[1]*imu->Cd[0]*imu->P_minus_Gama[0][0] + (1-imu->K_KF_Gama[1]*imu->Cd[1])*imu->P_minus_Gama[1][0];
    imu->P_plus_Gama[0][1] = (1-imu->K_KF_Gama[0]*imu->Cd[0])*imu->P_minus_Gama[0][1] - imu->K_KF_Gama[0]*imu->Cd[1]*imu->P_minus_Gama[1][1];
    imu->P_plus_Gama[1][1] = -imu->K_KF_Gama[1]*imu->Cd[0]*imu->P_minus_Gama[0][1] + (1-imu->K_KF_Gama[1]*imu->Cd[1])*imu->P_minus_Gama[1][1];

    imu->P_minus_Gama[0][0] = (imu->Ad[0][0]*imu->P_plus_Gama[0][0] + imu->Ad[0][1]*imu->P_plus_Gama[1][0])*imu->Ad[0][0]
							 +(imu->Ad[0][0]*imu->P_plus_Gama[0][1] + imu->Ad[0][1]*imu->P_plus_Gama[1][1])*imu->Ad[0][1]
					         +imu->R_w_Gama*(imu->Gd[0]*imu->Gd[0]);
    imu->P_minus_Gama[1][0] = (imu->Ad[1][0]*imu->P_plus_Gama[0][0] + imu->Ad[1][1]*imu->P_plus_Gama[1][0])*imu->Ad[0][0]
							 +(imu->Ad[1][0]*imu->P_plus_Gama[0][1] + imu->Ad[1][1]*imu->P_plus_Gama[1][1])*imu->Ad[0][1]
					         +imu->R_w_Gama*(imu->Gd[1]*imu->Gd[0]);
    imu->P_minus_Gama[0][1] = (imu->Ad[0][0]*imu->P_plus_Gama[0][0] + imu->Ad[0][1]*imu->P_plus_Gama[1][0])*imu->Ad[1][0]
							 +(imu->Ad[0][0]*imu->P_plus_Gama[0][1] + imu->Ad[0][1]*imu->P_plus_Gama[1][1])*imu->Ad[1][1]
					         +imu->R_w_Gama*(imu->Gd[0]*imu->Gd[1]);
    imu->P_minus_Gama[1][1] = (imu->Ad[1][0]*imu->P_plus_Gama[0][0] + imu->Ad[1][1]*imu->P_plus_Gama[1][0])*imu->Ad[1][0]
							 +(imu->Ad[1][0]*imu->P_plus_Gama[0][1] + imu->Ad[1][1]*imu->P_plus_Gama[1][1])*imu->Ad[1][1]
					         +imu->R_w_Gama*(imu->Gd[1]*imu->Gd[1]);

    imu->x_Gama_KF_p[0] = imu->Ad[0][0]*imu->x_Gama_KF_c_past[0] + imu->Ad[0][1]*imu->x_Gama_KF_c_past[1] + imu->Gyro_Gama;//imu->Bd[0]*imu->Gyro_Gama;
    imu->x_Gama_KF_p[1] = imu->Ad[1][0]*imu->x_Gama_KF_c_past[0] + imu->Ad[1][1]*imu->x_Gama_KF_c_past[1] + imu->Bd[1]*imu->Gyro_Gama;



}

void imu_filter_data(IMU *imu)
{
	/********************************************************************/
	/* 					Obtenção de dados filtrados						*/
	/********************************************************************/

	//Filtragem do acelerômetro por filtro passa baixa
	imu->Acc_X_f=	(imu->Acc_X*imu->Acc_DLPF_A[0] 			+	imu->Acc_X_p[0]*imu->Acc_DLPF_A[1]		+ imu->Acc_X_p[1]*imu->Acc_DLPF_A[2]
					-imu->Acc_X_f_p[0]*imu->Acc_DLPF_B[1]	-	imu->Acc_X_f_p[1]*imu->Acc_DLPF_B[2])
							/imu->Acc_DLPF_B[0];

	imu->Acc_Y_f=	(imu->Acc_Y*imu->Acc_DLPF_A[0] 			+ 	imu->Acc_Y_p[0]*imu->Acc_DLPF_A[1]		+ imu->Acc_Y_p[1]*imu->Acc_DLPF_A[2]
					-imu->Acc_Y_f_p[0]*imu->Acc_DLPF_B[1]	-	imu->Acc_Y_f_p[1]*imu->Acc_DLPF_B[2])
							/imu->Acc_DLPF_B[0];

	imu->Acc_Z_f=	(imu->Acc_Z*imu->Acc_DLPF_A[0] 			+ 	imu->Acc_Z_p[0]*imu->Acc_DLPF_A[1]		+ imu->Acc_Z_p[1]*imu->Acc_DLPF_A[2]
					-imu->Acc_Z_f_p[0]*imu->Acc_DLPF_B[1]	-	imu->Acc_Z_f_p[1]*imu->Acc_DLPF_B[2])
							/imu->Acc_DLPF_B[0];

	//Filtragem do giroscópio por filtro passa baixa
	imu->Gyro_X_f=	(imu->Gyro_X*imu->Gyro_DLPF_A[0] 		+	imu->Gyro_X_p[0]*imu->Gyro_DLPF_A[1]	+ imu->Gyro_X_p[1]*imu->Gyro_DLPF_A[2]
					-imu->Gyro_X_f_p[0]*imu->Gyro_DLPF_B[1]	-	imu->Gyro_X_f_p[1]*imu->Gyro_DLPF_B[2])
							/imu->Gyro_DLPF_B[0];

	imu->Gyro_Y_f=	(imu->Gyro_Y*imu->Gyro_DLPF_A[0] 		+	imu->Gyro_Y_p[0]*imu->Gyro_DLPF_A[1]	+ imu->Gyro_Y_p[1]*imu->Gyro_DLPF_A[2]
					-imu->Gyro_Y_f_p[0]*imu->Gyro_DLPF_B[1]	-	imu->Gyro_Y_f_p[1]*imu->Gyro_DLPF_B[2])
							/imu->Gyro_DLPF_B[0];

	imu->Gyro_Z_f=	(imu->Gyro_Z*imu->Gyro_DLPF_A[0] 		+	imu->Gyro_Z_p[0]*imu->Gyro_DLPF_A[1]	+ imu->Gyro_Z_p[1]*imu->Gyro_DLPF_A[2]
					-imu->Gyro_Z_f_p[0]*imu->Gyro_DLPF_B[1]	-	imu->Gyro_Z_f_p[1]*imu->Gyro_DLPF_B[2])
							/imu->Gyro_DLPF_B[0];

	//Filtragem do magnetômetro por filtro passa baixa
	imu->Mag_X_f=	(imu->Mag_X*imu->Mag_DLPF_A[0] 			+	imu->Mag_X_p[0]*imu->Mag_DLPF_A[1]		+ imu->Mag_X_p[1]*imu->Mag_DLPF_A[2]
					-imu->Mag_X_f_p[0]*imu->Mag_DLPF_B[1]	-	imu->Mag_X_f_p[1]*imu->Mag_DLPF_B[2])
							/imu->Mag_DLPF_B[0];

	imu->Mag_Y_f=	(imu->Mag_Y*imu->Mag_DLPF_A[0] 			+	imu->Mag_Y_p[0]*imu->Mag_DLPF_A[1]		+ imu->Mag_Y_p[1]*imu->Mag_DLPF_A[2]
					-imu->Mag_Y_f_p[0]*imu->Mag_DLPF_B[1]	-	imu->Mag_Y_f_p[1]*imu->Mag_DLPF_B[2])
							/imu->Mag_DLPF_B[0];

	imu->Mag_Z_f=	(imu->Mag_Z*imu->Mag_DLPF_A[0] 			+	imu->Mag_Z_p[0]*imu->Mag_DLPF_A[1]		+ imu->Mag_Z_p[1]*imu->Mag_DLPF_A[2]
					-imu->Mag_Z_f_p[0]*imu->Mag_DLPF_B[1]	-	imu->Mag_Z_f_p[1]*imu->Mag_DLPF_B[2])
							/imu->Mag_DLPF_B[0];

	imu->Mag_X_f=imu->Mag_X;
	imu->Mag_Y_f=imu->Mag_Y;
	imu->Mag_Z_f=imu->Mag_Z;

	//	imu->Gyro_X_f=	imu->Gyro_X;
	//
	//	imu->Gyro_Y_f=	imu->Gyro_Y;
	//
	//	imu->Gyro_Z_f=	imu->Gyro_Z;
}


void imu_get_initial_conditions(IMU *imu, int smpls)
{
	int i=0;
	float mean_X=0 ,mean_Y=0, mean_Z=0;
	float mean_Acc_X_BF=0, mean_Acc_Y_BF=0, mean_Acc_Z_BF=0;

	imu->gyro_offset_X=0;
	imu->gyro_offset_Y=0;
	imu->gyro_offset_Z=0;

	imu->Acc_X_NED_0=0;
	imu->Acc_Y_NED_0=0;
	imu->Acc_Z_NED_0=0;

	//Realiza uma quantidade de medidas com base na variável smpls
	while(i<smpls)
	{
		imu_get_data(imu);
		mean_X+=imu->Gyro_X;
		mean_Y+=imu->Gyro_Y;
		mean_Z+=imu->Gyro_Z;

		mean_Acc_X_BF+=imu->Acc_X_NED;
		mean_Acc_Y_BF+=imu->Acc_Y_NED;
		mean_Acc_Z_BF+=imu->Acc_Z_NED;
		i++;
	}
	//Condições iniciais
	imu->Acc_X_0=imu->Acc_X;
	imu->Acc_Y_0=imu->Acc_Y;
	imu->Acc_Z_0=imu->Acc_Z;

	imu->Gyro_X_0=imu->Gyro_X;
	imu->Gyro_Y_0=imu->Gyro_Y;
	imu->Gyro_Z_0=imu->Gyro_Z;

	imu->Mag_X_0=imu->Mag_X;
	imu->Mag_Y_0=imu->Mag_Y;
	imu->Mag_Z_0=imu->Mag_Z;

	imu->Acc_Phi_0=imu->Acc_Phi;;		imu->Gyro_Phi_0=imu->Gyro_Phi;;		imu->Mag_Phi_0=imu->Mag_Phi;
	imu->Acc_Theta_0=imu->Acc_Theta;	imu->Gyro_Theta_0=imu->Gyro_Theta;	imu->Mag_Theta_0=imu->Mag_Theta;
	imu->Acc_Gama_0=imu->Acc_Gama;		imu->Gyro_Gama_0=imu->Gyro_Gama;	imu->Mag_Gama_0=imu->Mag_Gama;

	//Offset do Giroscópio
	imu->gyro_offset_X=mean_X/smpls;
	imu->gyro_offset_Y=mean_Y/smpls;
	imu->gyro_offset_Z=mean_Z/smpls;

	//Offset Aceleração no BF
	imu->Acc_X_NED_0=mean_Acc_X_BF/smpls;
	imu->Acc_Y_NED_0=mean_Acc_Y_BF/smpls;
	imu->Acc_Z_NED_0=mean_Acc_Z_BF/smpls;

	imu->Vel_X_NED=0;
	imu->Vel_Y_NED=0;
	imu->Vel_Z_NED=0;

	imu->Pos_X_NED=0;
	imu->Pos_Y_NED=0;
	imu->Pos_Z_NED=0;

	imu->Gyro_Phi=0;
	imu->Gyro_Theta=0;
	imu->Gyro_Gama=0;
}

void imu_get_samples(IMU *imu, int time, int smp_p_sec, char* arq_name)
{
	float rate = 1000.0/(float)smp_p_sec;

	uint8_t imu_status;
	//sprintf(arq_name,"imuSamples%dSampPerSecFor%dSec.txt",smp_p_sec,time/1000);
	printf("Criando arquivo de nome:");
	printf(arq_name);
	printf("\r\n");

	FATFS       FatFs;                //Fatfs handle
	FIL         fil;                  //File handle
	FRESULT     fres;                 //Result after operations

	imu_status=imu->status;

	if(imu_status==7)
	{
		  do
		  {
		    //Mount the SD Card
		    fres = f_mount(&FatFs, "", 1);    //1=mount now
		    if (fres != FR_OK)
		    {
		      printf("No SD Card found : (%i)\r\n", fres);
		      break;
		    }
		    printf("SD Card Mounted Successfully!!!\r\n");
		    //Read the SD Card Total size and Free Size
		    FATFS *pfs;
		    DWORD fre_clust;
		    uint32_t totalSpace, freeSpace;
		    f_getfree("", &fre_clust, &pfs);
		    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
		    printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);
		    //Open the file
		    fres = f_open(&fil, arq_name, FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
		    if(fres != FR_OK)
		    {
		      printf("File creation/open Error : (%i)\r\n", fres);
		      break;
		    }
		    printf("Writing data!!!\r\n");
		    //write the data

		    printf("Esperado:%d\n",smp_p_sec*time/1000);

		    f_puts(
		    		"acc_x_raw,acc_y_raw,acc_z_raw"
//		    		",acc_x [g],acc_y [g],acc_z [g]"
		    		",gyro_x_raw,gyro_y_raw,gyro_z_raw"
//		    		",gyro_x [deg/s],gyro_y [deg/s],gyro_z [deg/s]"
//		    		",row [deg],pitch [deg],yaw [deg]"
		    		"\n"
		    		, &fil);

		    int count=0;
		    static char msg[60];

		    int procc_time=HAL_GetTick();
		    while(HAL_GetTick()<time+procc_time)
		    	  {
		    		  imu_get_data(imu);

		    		  sprintf(msg,
		    				  "%d,%d,%d"
//		    				  ",%f,%f,%f,"
		    				  ",%d,%d,%d"
//		    				  ",%f,%f,%f"
//		    				  ",%f,%f,%f"
		    				  "\n"
							  ,imu->raw_Acc_X,imu->raw_Acc_Y,imu->raw_Acc_Z
//							  ,imu->Acc_X,imu->Acc_Y,imu->Acc_Z
							  ,imu->raw_Gyro_X,imu->raw_Gyro_Y,imu->raw_Gyro_Z
//							  ,imu->Gyro_X,imu->Gyro_Y,imu->Gyro_Z
//							  ,imu->Phi,imu->Theta,imu->Acc_Yaw
							  );

		    		  f_puts(msg,&fil);

		    		  HAL_Delay(rate);
		    		  count++;
		    		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
		    	  }
		    printf("Amostras obtidas:%d\n",count);
		    //close your file
		    f_close(&fil);
		  } while(0);
		  //We're done, so de-mount the drive
		  f_mount(NULL, "", 0);
		  printf("SD Card Unmounted Successfully!!!\r\n");
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

	}else {return;}
}

/* Print Functions */

void imu_print_data(IMU* imu)
{
	  printf(
			  "\n\r"
			  "\n\r"
			  "Acc		X:%f	Y:%f	Z:%f	[g]"
			  "\n\r"
			  "\n\r"
			  "Gyro  		X:%f	Y:%f	Z:%f	[deg/s]"
			  "\n\r"
			  "\n\r"
			  "Mag  	 	X:%f	Y:%f	Z:%f	[uT]"
			  "\n\r"
			  ,imu->Acc_X,imu->Acc_Y,imu->Acc_Z
			  ,imu->Gyro_X,imu->Gyro_Y,imu->Gyro_Z
			  ,imu->Mag_X,imu->Mag_Y,imu->Mag_Z
			  );
}

void imu_print_filtered_data(IMU* imu)
{
	  printf(
			  "\n\r"
			  "\n\r"
			  "Acc_f		X:%f	Y:%f	Z:%f	[g]"
			  "\n\r"
			  "\n\r"
			  "Gyro_f		X:%f	Y:%f	Z:%f	[deg/s]"
			  "\n\r"
			  ,imu->Acc_X_f,imu->Acc_Y_f,imu->Acc_Z_f
			  ,imu->Gyro_X_f,imu->Gyro_Y_f,imu->Gyro_Z_f
			  );
}

void imu_print_raw_data(IMU* imu)
{
	  printf(
			  "\n\r"
			  "\n\r"
			  "Raw Acc  	X:%d	Y:%d	Z:%d"
			  "\n\r"
			  "Raw Gyro 	X:%d	Y:%d	Z:%d"
			  "\n\r"
			  "Raw Mag 		X:%d	Y:%d	Z:%d"
			  "\n\r"
			  ,imu->raw_Acc_X,imu->raw_Acc_Y,imu->raw_Acc_Z
			  ,imu->raw_Gyro_X,imu->raw_Gyro_Y,imu->raw_Gyro_Z
			  ,imu->raw_Mag_X,imu->raw_Mag_Y,imu->raw_Mag_Z
			  );
}

void imu_print_BF_info(IMU* imu)
{
	  printf(
			  "\n\r"
			  "\n\r"
			  "Acc  		X:%f	Y:%f	Z:%f"
			  "\n\r"
			  "Vel		X:%f	Y:%f	Z:%f"
			  "\n\r"
			  "Pos 		X:%f	Y:%f	Z:%f"
			  "\n\r"
			  ,imu->Acc_X_NED,imu->Acc_Y_NED,imu->Acc_Z_NED
			  ,imu->Vel_X_NED,imu->Vel_Y_NED,imu->Vel_Z_NED
			  ,imu->Pos_X_NED,imu->Pos_Y_NED,imu->Pos_Z_NED
			  );
}

void imu_print_att_data(IMU* imu)
{
	  printf(
			  "\n\r"
			  "\n\r"
			  "Acc :	Phi:%f		Theta:%f		Gama:%f"
			  "\n\r"
			  "Gyro: 	Phi:%f		Theta:%f		Gama:%f"
			  "\n\r"
			  "Mag : 	Phi:%f		Theta:%f		Gama:%f"
			  "\n\r"
			  "Est : 	Phi:%f		Theta:%f		Gama:%f"
			  "\n\r"
			  ,imu->Acc_Phi*180/M_PI,imu->Acc_Theta*180/M_PI,imu->Acc_Gama*180/M_PI
			  ,imu->Gyro_Phi*180/M_PI,imu->Gyro_Theta*180/M_PI,imu->Gyro_Gama*180/M_PI
			  ,imu->Mag_Phi*180/M_PI,imu->Mag_Theta*180/M_PI,imu->Mag_Gama*180/M_PI
			  ,imu->Phi*180/M_PI,imu->Theta*180/M_PI,imu->Gama*180/M_PI
			  );
}

void imu_print_att_data2(IMU* imu)
{
	  printf(
			  "\n\r"
			  "\n\r"
			  "CF :	Phi:%f		Theta:%f		Gama:%f"
			  "\n\r"
			  "SO: 	Phi:%f		Theta:%f		Gama:%f"
			  "\n\r"
			  "KF : 	Phi:%f		Theta:%f		Gama:%f"
			  "\n\r"
			  "____________________________________________________________"
			  "\n\r"
			  ,imu->Phi_CF*180/M_PI,imu->Theta_CF*180/M_PI,imu->Gama_CF*180/M_PI
			  ,imu->Phi_SO*180/M_PI,imu->Theta_SO*180/M_PI,imu->Gama_SO*180/M_PI
			  ,imu->Phi_KF*180/M_PI,imu->Theta_KF*180/M_PI,imu->Gama_KF*180/M_PI
			  );
}

void imu_print_data_matlab_raw_and_filtered(IMU* imu)
{
	  printf(
			  "%f %f %f "
			  "%f %f %f "
			  "%f %f %f "
			  "%f %f %f "
			  "%f %f %f "
			  "%f %f %f "
			  "%f"
			  "\n"
			  ,imu->Acc_X_f,imu->Acc_Y_f,imu->Acc_Z_f
			  ,imu->Acc_X,imu->Acc_Y,imu->Acc_Z
			  ,imu->Gyro_X_f,imu->Gyro_Y_f,imu->Gyro_Z_f
			  ,imu->Gyro_X,imu->Gyro_Y,imu->Gyro_Z
			  ,imu->Mag_X_f,imu->Mag_Y_f,imu->Mag_Z_f
			  ,imu->Mag_X,imu->Mag_Y,imu->Mag_Z
			  ,imu->curr_Time
			  );
}

void imu_print_data_matlab_attitude(IMU* imu)
{
	  printf(
			  "%f %f %f "
			  "%f %f %f "
			  "%f %f %f "
			  "%f %f %f "
			  "%f %f %f "
			  "%f"
			  "\n"
			  ,imu->Acc_Phi*180/M_PI,imu->Acc_Theta*180/M_PI,imu->Mag_Gama*180/M_PI
			  ,imu->Gyro_Phi*180/M_PI,imu->Gyro_Theta*180/M_PI,imu->Gyro_Gama*180/M_PI
			  ,imu->Phi_CF*180/M_PI,imu->Theta_CF*180/M_PI,imu->Gama_CF*180/M_PI
			  ,imu->Phi_SO*180/M_PI,imu->Theta_SO*180/M_PI,imu->Gama_SO*180/M_PI
			  ,imu->Phi_KF*180/M_PI,imu->Theta_KF*180/M_PI,imu->Gama_KF*180/M_PI
			  ,imu->curr_Time
			  );
}

void imu_print_data_matlab_euler_n_pos(IMU* imu)
{
	  printf(
			  "%f %f %f "
			  "%f %f %f "
			  "%f"
			  "\n"
			  ,imu->Phi*180/M_PI,imu->Theta*180/M_PI,imu->Gama*180/M_PI
			  ,imu->Pos_X_NED,imu->Pos_Y_NED,imu->Pos_Z_NED
			  ,imu->curr_Time
			  );
}

void imu_print_mag_data_matlab(IMU* imu)
{
	  printf(
			  "%d %d %d"
			  "\n"
			  ,imu->raw_Mag_X,imu->raw_Mag_Y,imu->raw_Mag_Z
			  );
}

/* Read and Write Functions */

void imu_Write_Reg(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(CS_IMU_GPIO, CS_IMU_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&imuspi, &reg, 1, 100);
	HAL_SPI_Transmit(&imuspi, &data, 1, 100);

//	HAL_SPI_Transmit_DMA(&imuspi, &reg, 1);
//	HAL_SPI_Transmit_DMA(&imuspi, &data, 1);

//	HAL_SPI_Transmit_IT(&imuspi, &reg, 1);
//	HAL_SPI_Transmit_IT(&imuspi, &data, 1);

	HAL_GPIO_WritePin(CS_IMU_GPIO, CS_IMU_Pin, GPIO_PIN_SET);
}

void imu_Read_Reg(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t temp_data = 0x80|reg;
	HAL_GPIO_WritePin(CS_IMU_GPIO, CS_IMU_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&imuspi, &temp_data, 1, 100);
	HAL_SPI_Receive(&imuspi, data, len, 100);

//	HAL_SPI_Transmit_DMA(&imuspi, &temp_data, 1);
//	HAL_SPI_Receive_DMA(&imuspi, data, len);
//
//	HAL_SPI_Transmit_IT(&imuspi, &temp_data, 1);
//	HAL_SPI_Receive_IT(&imuspi, data, len);


	HAL_GPIO_WritePin(CS_IMU_GPIO, CS_IMU_Pin, GPIO_PIN_SET);
}

/************************MAG Functions************************/

/*Configuration Functions */
void mag_begin(IMU* imu)
{
	uint8_t val=0;

	/* Reseta I2C Master*/
	val=0;
	imu_bank(0, imu);
	imu_Read_Reg(ICM20948_USER_CTRL,&val,1);
	val|=0x02;
	imu_Write_Reg(ICM20948_USER_CTRL,val);
	HAL_Delay(100);

	/* Habilita I2C Master */
	val=0;
	imu_Read_Reg(ICM20948_USER_CTRL,&val,1);
	val|=0x20;
	imu_Write_Reg(ICM20948_USER_CTRL,val);
	HAL_Delay(10);

	/* Configura clock da comunicação */
	val=0;
	imu_bank(3, imu);
	val=0x07;
	imu_Write_Reg(ICM20948_I2C_MST_CTRL, val);
	HAL_Delay(10);

	/* Configuração durante LP */
	val=0;
	imu_bank(0, imu);
	val=0x40;
	imu_Write_Reg(ICM20948_LP_CONFIG, val);
	HAL_Delay(10);

	/* Configuração de ODR */
	val=0;
	imu_bank(3, imu);
	val=0x03;
	imu_Write_Reg(ICM20948_I2C_MST_ODR_CFG, val);
	HAL_Delay(10);

	/* Reseta AK09916 */
	val=0;
	val=0x01;
	mag_Write_Reg(AK09916_CNTL_3, val, imu);
	HAL_Delay(100);
	mag_Write_Reg(AK09916_CNTL_3, val, imu);
	HAL_Delay(100);
	val=0;
	mag_Write_Reg(AK09916_CNTL_2, val, imu);
	HAL_Delay(100);

	/* Modo continuo de transmissão */
	val=0;
	val=0x08;
	mag_Write_Reg(AK09916_CNTL_2, val, imu);
	HAL_Delay(100);
}

/* Read and Write Functions */
void mag_Write_Reg(uint8_t reg, uint8_t data, IMU* imu)
{
	imu_bank(3, imu);

	imu_Write_Reg(ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS);
	HAL_Delay(10);
	imu_Write_Reg(ICM20948_I2C_SLV0_REG, reg);
	HAL_Delay(10);
	imu_Write_Reg(ICM20948_I2C_SLV0_DO, data);
	HAL_Delay(10);
	imu_Write_Reg(ICM20948_I2C_SLV0_CTRL, 0x80|0x01);
	HAL_Delay(100);

	imu_bank(0, imu);
}

void mag_Read_Reg(uint8_t reg, uint8_t len, IMU* imu)
{
	imu_bank(3, imu);

	imu_Write_Reg(ICM20948_I2C_SLV0_ADDR, 0x80|AK09916_ADDRESS);
	HAL_Delay(10);
	imu_Write_Reg(ICM20948_I2C_SLV0_REG, reg);
	HAL_Delay(10);
	imu_Write_Reg(ICM20948_I2C_SLV0_CTRL, 0x80|len);
	HAL_Delay(100);

	imu_bank(0, imu);
}

