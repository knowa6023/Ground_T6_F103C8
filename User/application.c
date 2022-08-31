/*
 * application.c
 *
 *  Created on: Feb 28, 2021
 *      Author: PC
 */

#include "application.h"
#include "main.h"
#include "timer.h"
#include "mcu_communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*----------------------------------local------------------------------------------*/
/*----------------------------------varibles---------------------------------------*/
static Timer ledTimer;
static Timer tickTimer;
static uint8_t cmdButton_1[] ="<FM>PUSH_BUTTON:1<END>\r\n";
static uint8_t msg_Hello[] ="Ground_T6 ver 1.1 \r\n";
static char buff[100]={0};
static uint8_t number_temp=1;
static double result,result1,result2,result4=0;
static 	uint32_t par;
static uint8_t dataUART;

static const double t10=1039.0;
static const double t20=1077.9;
static const double t25=1097.4;
static  double res,t,tmax=-100,tmin=100,tdelta=0,time=0,tdelta_of_1_min=0;
static unsigned char buf[29];//18
//static char buff[27]={0};


/*----------------------------------prototypes function-----------------------------*/
void eAPP_StartMCU_UART_Receive( void );
void eAPP_UART_Transmit_IT(uint8_t str[]);
static void camera_cmd_callback(CMD_Type cmd, const CAMARG *args );

static void chipSelect_1_LOW();
static void chipSelect_2_LOW();
static void chipSelect_1_HIGH();
static void chipSelect_2_HIGH();
static int32_t Convert_R_to_Temperature(double R);
extern void Select_SPI_Device(uint8_t i);
/*----------------------------------local-------------------------------------------*/





/*----------------------------------extern-----------------------------------------*/
/*----------------------------------varibles---------------------------------------*/
extern UART_HandleTypeDef huart1;

/*----------------------------------prototypes function----------------------------*/

/*----------------------------------extern-----------------------------------------*/

PT1000_t PT1000;
void App_main()
{
	TMR_Init();
	// Обычный секундный беспонтовый светодиод
	TMR_Add( &ledTimer, ledTimer_Callback, TMR_RELOAD_YES );
	TMR_Start( &ledTimer, 1000);

	CAMCMD_Init(camera_cmd_callback);
	eAPP_StartMCU_UART_Receive();
	HAL_GPIO_WritePin(USART1_DE_GPIO_Port, USART1_DE_Pin, SET);

	eAPP_UART_Transmit_IT(msg_Hello);
	while(1)
	{
		TMR_ExecuteCallbacks();
		//CAMCMD_ProcessMessages();
		//eAPP_UART_Transmit_IT(cmdButton_1);
///*********************************************************************************************
//1*********************************************************************************************
///*********************************************************************************************
		AD7792_Reset();
		AD7792_Init(1);
		Select_SPI_Device(1);
		PT1000.mas[0]=-9999;
		PT1000.mas[1]=-9999;
		AD7792_Config_(1);
		par= AD7792_ContinuousReadAvg(250,1);
		result=(((double)par/0xFFFFFF))*4020;///mv
		PT1000.mas[0]=(uint32_t)(10*result);
		PT1000.mas[1]= Convert_R_to_Temperature(result);
		HAL_Delay(10);
///*********************************************************************************************
//2*********************************************************************************************
///*********************************************************************************************
		AD7792_Reset();
		AD7792_Init(1);
		Select_SPI_Device(2);
		PT1000.mas[2]=-9999;
		PT1000.mas[3]=-9999;
		//AD7792_Config(1,2);
		AD7792_Config_(1);
		par= AD7792_ContinuousReadAvg(250,1);
		result=(((double)par/0xFFFFFF))*4020;///mv
		PT1000.mas[2]=(uint32_t)(10*result);
		PT1000.mas[3]= Convert_R_to_Temperature(result);
		HAL_Delay(10);
///*********************************************************************************************
//3*********************************************************************************************
///*********************************************************************************************
		AD7792_Reset();
		AD7792_Init(1);
		Select_SPI_Device(3);
		PT1000.mas[4]=-9999;
		PT1000.mas[5]=-9999;
		AD7792_Config_(1);
		par= AD7792_ContinuousReadAvg(250,1);
		result=(((double)par/0xFFFFFF))*4020;///mv
		PT1000.mas[4]=(uint32_t)(10*result);
		PT1000.mas[5]= Convert_R_to_Temperature(result);
		HAL_Delay(10);
///*********************************************************************************************
//4*********************************************************************************************
///*********************************************************************************************
		AD7792_Reset();
		AD7792_Init(1);
		Select_SPI_Device(4);
		PT1000.mas[6]=-9999;
		PT1000.mas[7]=-9999;
		AD7792_Config_(1);
		par= AD7792_ContinuousReadAvg(250,1);
		result=(((double)par/0xFFFFFF))*4020;///mv
		PT1000.mas[6]=(uint32_t)(10*result);
		PT1000.mas[7]= Convert_R_to_Temperature(result);
		HAL_Delay(10);
///*********************************************************************************************
//5*********************************************************************************************
///*********************************************************************************************
		AD7792_Reset();
		AD7792_Init(1);
		Select_SPI_Device(5);
		PT1000.mas[8]=-9999;
		PT1000.mas[9]=-9999;
		AD7792_Config_(1);
		par= AD7792_ContinuousReadAvg(250,1);
		result=(((double)par/0xFFFFFF))*4020;///mv
		PT1000.mas[8]=(uint32_t)(10*result);
		PT1000.mas[9]= Convert_R_to_Temperature(result);
		HAL_Delay(10);
///*********************************************************************************************
//6*********************************************************************************************
///*********************************************************************************************
		AD7792_Reset();
		AD7792_Init(1);
		Select_SPI_Device(6);
		PT1000.mas[10]=-9999;
		PT1000.mas[11]=-9999;
		AD7792_Config_(1);
		par= AD7792_ContinuousReadAvg(250,1);
		result=(((double)par/0xFFFFFF))*4020;///mv
		PT1000.mas[10]=(uint32_t)(10*result);
		PT1000.mas[11]= Convert_R_to_Temperature(result);
		HAL_Delay(10);
///*********************************************************************************************
///*********************************************************************************************
///*********************************************************************************************
		sprintf(buff, "<FM>Temperature T1:%03d 	T2:%03d	T3:%03d	T4:%03d	T5:%03d	T6:%03d, <END>\r\n", PT1000.mas[1],PT1000.mas[3],PT1000.mas[5],PT1000.mas[7],PT1000.mas[9],PT1000.mas[11]);
		eAPP_UART_Transmit_IT(buff);
	}

}

void ledTimer_Callback(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	time++;
//	if (time==60)
//	{
//		time=0;
//		tdelta_of_1_min=tdelta;
//		tmax=-100;
//		tmin=100;
//	}
}

static void camera_cmd_callback(CMD_Type cmd, const CAMARG *args ){

	//TMR_Restart(&doNothingResetTimer);
	switch(cmd){

		case CMD_LAMP_ON:
HAL_Delay(100);
			break;
		case CMD_LAMP_OFF:
			HAL_Delay(100);
			break;
		default:

			break;
	}
}

static uint8_t data;
void eAPP_StartMCUReceive( void ){
	HAL_UART_Receive_IT( &huart1, &data, 1 );
}
void eAPP_StartMCU_UART_Receive( void ){
	HAL_UART_Receive_IT( &huart1, &dataUART, 1 );
}
void eAPP_UART_Transmit_IT(uint8_t str[]){
	HAL_UART_Transmit_IT(&huart1, str,strlen(str));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart1)
    {
		MCU_HandleByte(dataUART);
    }

	eAPP_StartMCU_UART_Receive();
}


static void chipSelect_1_LOW()
{
	HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, RESET);
}
static void chipSelect_2_LOW()
{
	HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, RESET);
}

static void chipSelect_1_HIGH()
{
	HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
}
static void chipSelect_2_HIGH()
{
	HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
}

void eAPP_Chip_Select()
{
	HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, RESET);
	//HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, RESET);

}
void eAPP_Chip_Un_Select()
{
	HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
	//HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
}

int32_t Convert_R_to_Temperature(double R)
{
   // int32_t res=-9999;

    static float table_Temperature[]={-50,-40,-30,-20,-10,0,10,20,30,40,50,60,70,80,90,100};
    static float table_R[]={803.15,842.75, 882.24, 921.61, 960.86, 1000.0,1039.02, 1077.93, 1116.72, 1155.39, 1193.95, 1232.39, 1270.72, 1308.93, 1347.02, 1385.00};
    uint8_t index=0;
    int8_t buf_index=-1;
    uint8_t buf=sizeof(table_R);
    static float k=1;
    for(uint8_t i=0;i<15;i++)
    {
    	if(table_R[i]>=R)
    	{
    		buf_index=i-1;
    		break;
    	}
    }

    //разбиение индекса
    if (buf_index<15)
    {
    	k=((float)(table_Temperature[buf_index+1]-table_Temperature[buf_index])/(table_R[buf_index+1]-table_R[buf_index]));

    	//res=(int32_t)((k*((float)R-table_R[buf_index]))*10);

    	res= (int32_t)((k*(R-table_R[buf_index])+table_Temperature[buf_index])*100);
    }
    else
    {
    	res=1000;
    }

	return res;
}
