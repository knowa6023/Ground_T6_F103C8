/*
 * AD7792.c
 *
 *  Created on: Mar 1, 2021
 *      Author: PC
 */


/***************************************************************************//**
 *   @file   AD7792.c
 *   @brief  Implementation of AD7792 Driver.
 *   @author Bancisor MIhai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 500
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7792.h"				// AD7792 definitions.
#include "SPI_communication.h"		// Communication definitions.
#include "main.h"
//#include "platform.h"

#include <stdio.h>
#include <string.h>



extern void eAPP_Chip_Un_Select();
extern void eAPP_Chip_Select();
extern SPI_HandleTypeDef hspi1;

#define POLAR_MODE   1
#define DELAY_FOR_DATAREADY_MS 10
//#define BIPOLAR_MODE 0


static char buf[40];//18

/***************************************************************************//**
 * @brief Initializes the AD7792 and checks if the device is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char AD7792_Init(uint8_t i)
{
	uint8_t status = 0x1;

	uint8_t byteTx = AD7792_COMM_READ|AD7792_COMM_ADDR(AD7792_REG_ID);
	uint8_t byteRx;

	//Select_SPI_Device(i);
	HAL_SPI_Transmit(&hspi1, &byteTx, 1, 0xFFFF);
	HAL_SPI_Receive(&hspi1, &byteRx, 1, 0xFFFF);
	//Select_SPI_Device(0);

	if (((byteRx & 0xF)!=AD7792_ID)){
		status = 0x0;
	}

	return(status);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @return  None.
*******************************************************************************/
void AD7792_Reset(void)
{
	uint8_t dataToSend[6] = {0x03, 0xff, 0xff, 0xff, 0xff, 0xff};
   // ADI_PART_CS_LOW;
    Select_SPI_Device(1);
	SPI_Write(dataToSend,5);
	//ADI_PART_CS_HIGH;
    Select_SPI_Device(0);


    HAL_Delay(10);
    Select_SPI_Device(2);
	SPI_Write(dataToSend,5);
	//ADI_PART_CS_HIGH;
    Select_SPI_Device(0);

    HAL_Delay(10);
    Select_SPI_Device(3);
	SPI_Write(dataToSend,5);
	//ADI_PART_CS_HIGH;
    Select_SPI_Device(0);


    HAL_Delay(10);
    Select_SPI_Device(4);
	SPI_Write(dataToSend,5);
	//ADI_PART_CS_HIGH;
    Select_SPI_Device(0);


    HAL_Delay(10);
    Select_SPI_Device(5);
	SPI_Write(dataToSend,5);
	//ADI_PART_CS_HIGH;
    Select_SPI_Device(0);


    HAL_Delay(10);
    Select_SPI_Device(6);
	SPI_Write(dataToSend,5);
	//ADI_PART_CS_HIGH;
    Select_SPI_Device(0);

}
/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
unsigned long AD7792_GetRegisterValue(unsigned char regAddress,
                                      unsigned char size)
{
	unsigned char data[5]      = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned long receivedData = 0x00;
    unsigned char i            = 0x00;
    uint8_t buf =AD7792_COMM_READ |  AD7792_COMM_ADDR(regAddress);

	SPI_Write(&buf,1);
	SPI_Read(&data,(size));
	for(i = 0;i < size ;i ++)
    {
        receivedData = (receivedData << 8) + data[i];
    }

    return (receivedData);
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.
*******************************************************************************/
void AD7792_SetRegisterValue(unsigned char regAddress,
                             unsigned long regValue,
                             unsigned char size)
{
	unsigned char data[5]      = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned char* dataPointer = (unsigned char*)&regValue;
    unsigned char bytesNr      = size ;

    data[0] = AD7792_COMM_WRITE |  AD7792_COMM_ADDR(regAddress);
    while(bytesNr > 0)
    {
        data[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }
	SPI_Write(data,(size+1));
}
/***************************************************************************//**
 * @brief  Waits for RDY pin to go low.
 *
 * @return None.
*******************************************************************************/
void AD7792_WaitRdyGoLow(void)
{
	uint32_t  timme = 0;
    while( AD7792_RDY_STATE || (HAL_GetTick()-timme)>DELAY_FOR_DATAREADY_MS)
    {
    	timme = HAL_GetTick();
    }
}

/***************************************************************************//**
 * @brief Sets the operating mode of AD7792.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.
*******************************************************************************/
void AD7792_SetMode(unsigned long mode)
{
    unsigned long command;

    command = AD7792_GetRegisterValue(AD7792_REG_MODE,
                                      2);
    command &= ~AD7792_MODE_SEL(0xFF);
    command |= AD7792_MODE_SEL(mode);
    AD7792_SetRegisterValue(
            AD7792_REG_MODE,
            command,
            2);
}
/***************************************************************************//**
 * @brief Selects the channel of AD7792.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.
*******************************************************************************/
void AD7792_SetChannel(unsigned long channel)
{
    unsigned long command;

    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2);
    command &= ~AD7792_CONF_CHAN(0xFF);
    command |= AD7792_CONF_CHAN(channel);
    AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2);
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.
*******************************************************************************/
void AD7792_SetGain(unsigned long gain)
{
    unsigned long command;

    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2);
    command &= ~AD7792_CONF_GAIN(0xFF);
    command |= AD7792_CONF_GAIN(gain);
    AD7792_SetRegisterValue(
            AD7792_REG_CONF,
            command,
            2);
}
/***************************************************************************//**
 * @brief Sets the reference source for the ADC.
 *
 * @param type - Type of the reference.
 *               Example: AD7792_REFSEL_EXT	- External Reference Selected
 *                        AD7792_REFSEL_INT	- Internal Reference Selected.
 *
 * @return None.
*******************************************************************************/
void AD7792_SetIntReference(unsigned char type)
{
    unsigned long command = 0;

    command = AD7792_GetRegisterValue(AD7792_REG_CONF,
                                      2);
    command &= ~AD7792_CONF_REFSEL(AD7792_REFSEL_INT);
    command |= AD7792_CONF_REFSEL(type);
    AD7792_SetRegisterValue(AD7792_REG_CONF,
							command,
							2);
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7792_Calibrate(unsigned char mode, unsigned char channel)
{
    unsigned short oldRegValue = 0x0;
    unsigned short newRegValue = 0x0;

    AD7792_SetChannel(channel);
    oldRegValue &= AD7792_GetRegisterValue(AD7792_REG_MODE, 2); // CS is modified by SPI read/write functions.
    oldRegValue &= ~AD7792_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7792_MODE_SEL(mode);
    ADI_PART_CS_LOW;
    AD7792_SetRegisterValue(AD7792_REG_MODE, newRegValue, 2); // CS is not modified by SPI read/write functions.
    AD7792_WaitRdyGoLow();
    ADI_PART_CS_HIGH;

}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned long AD7792_SingleConversion(void)
{
    unsigned long command = 0x0;
    unsigned long regData = 0x0;

    command  = AD7792_MODE_SEL(AD7792_MODE_SINGLE);
    ADI_PART_CS_LOW;
    AD7792_SetRegisterValue(AD7792_REG_MODE,
                            command,
                            2);// CS is not modified by SPI read/write functions.
    AD7792_WaitRdyGoLow();
    regData = AD7792_GetRegisterValue(AD7792_REG_DATA, 3); // CS is not modified by SPI read/write functions.
   // ADI_PART_CS_HIGH;

    return(regData);
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned long AD7792_ContinuousReadAvg(uint16_t sampleNumber,uint8_t i)
{
    unsigned long samplesAverage = 0x0;
    unsigned long command        = 0x0;
    uint16_t count          	 = 0x0;

    command = AD7792_MODE_SEL(AD7792_MODE_CONT);
    //Select_SPI_Device(i);
  //  ADI_PART_CS_LOW;
    AD7792_SetRegisterValue(AD7792_REG_MODE,
                            command,
                            2);
    for(count = 0;count < sampleNumber;count ++)
    {
        AD7792_WaitRdyGoLow();
        samplesAverage += AD7792_GetRegisterValue(AD7792_REG_DATA, 3);  // CS is not modified by SPI read/write functions.
    }
    //ADI_PART_CS_HIGH;
    //Select_SPI_Device(0);
    samplesAverage = samplesAverage / sampleNumber;

    return(samplesAverage);
}



/***************************************************************************//**
 * @.
 *
 *
 *Так как в данный момент установленный эталонный резистор не подходит, будем использовать внутренний эталонный источник напряжения
*******************************************************************************/
uint8_t AD7792_Config(uint8_t mode_polar,uint8_t i){
	uint8_t status= 0x0;

	//uint8_t CR_for_IO_Register	 = AD7792_COMM_ADDR(AD7792_REG_IO)|AD7792_COMM_WRITE;
	//uint8_t IO_Register			 = AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2)|AD7792_IEXCEN(AD7792_EN_IXCEN_210uA);
	uint8_t IO_Register			 = AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2)|AD7792_IEXCEN(AD7792_EN_IXCEN_210uA);//AD7792_EN_IXCEN_10uA
	//uint32_t buf;
	//uint32_t par;
	//float result=0;
	//const float Vref=1.113240;	uint8_t IO_Register			 = AD7793_IEXCDIR(AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2)|AD7793_IEXCEN(AD7793_EN_IXCEN_210uA);
	//const uint8_t gain =8;
	//const double k= 139.155;//Vref/gain
    uint16_t TxConfigReg;// =AD7792_CONF_POLAR|AD7792_CONF_GAIN(AD7792_GAIN_8);
	//buf=0;
	//result=0;
	HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS1_Pin, RESET);
	HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS2_Pin, SET);
	HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
	HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
	HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
	HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);
	HAL_Delay(10);

	//AD7792_Reset();
	if (AD7792_Init(i)){
		//eAPP_Chip_Select();
//	HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, RESET);
//	HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
//	HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
//	HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
//	HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
//	HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);

		//Select_SPI_Device(i);
		if (mode_polar==1){

			//TxConfigReg =AD7792_CONF_UNIPOLAR|AD7792_CONF_GAIN(AD7792_GAIN_1);
			//AD7792_SetMode(AD7792_MODE_SEL(AD7792_MODE_CONT)|AD7792_MODE_CLKSRC(AD7792_CLK_INT)|/*AD7792_MODE_RATE(0x2)*/0xA);
			//AD7792_SetIntReference(AD7792_REFSEL_EXT);
			//AD7792_SetChannel(AD7792_CH_AIN2P_AIN2M);
			//AD7792_SetRegisterValue(AD7792_REG_IO, IO_Register, 1);	// установка генератора тока


			TxConfigReg =AD7792_CONF_UNIPOLAR|AD7792_CONF_GAIN(AD7792_GAIN_1);
			AD7792_SetMode(AD7792_MODE_SEL(AD7792_MODE_CONT)|AD7792_MODE_CLKSRC(AD7792_CLK_INT)|/*AD7793_MODE_RATE(0x2)*/0xA);
			AD7792_SetIntReference(AD7792_REFSEL_EXT);
			AD7792_SetChannel(AD7792_CH_AIN1P_AIN1M);
			AD7792_SetRegisterValue(AD7792_REG_CONF, TxConfigReg, 2);// установка конфигурации
			AD7792_SetRegisterValue(AD7792_REG_IO, IO_Register, 1);	// установка генератора тока


		}
		else{
				TxConfigReg =AD7792_CONF_BIPOLAR|AD7792_CONF_GAIN(AD7792_GAIN_1);
				AD7792_SetMode(AD7792_MODE_SEL(AD7792_MODE_CONT)|AD7792_MODE_CLKSRC(AD7792_CLK_INT)|/*AD7792_MODE_RATE(0x2)*/0xA);
				AD7792_SetIntReference(AD7792_REFSEL_EXT);///AD7792_REFSEL_EXT
				AD7792_SetChannel(AD7792_CH_AIN1P_AIN1M);
				AD7792_SetRegisterValue(AD7792_REG_CONF, TxConfigReg, 2);
				AD7792_SetRegisterValue(AD7792_REG_IO, IO_Register, 1);				// установка генератора тока
		}
		status=0x01;
	}
	else{
		//eAPP_LedOff();
		//AD7792_Reset();
	}
	//Select_SPI_Device(0);
	return (status);
}
uint8_t AD7792_Config_(uint8_t mode_polar){
	uint8_t status= 0x0;
	uint8_t IO_Register			 = AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2)|AD7792_IEXCEN(AD7792_EN_IXCEN_210uA);//AD7792_EN_IXCEN_10uA
    uint16_t TxConfigReg;// =AD7792_CONF_POLAR|AD7792_CONF_GAIN(AD7792_GAIN_8);

	if (mode_polar==1){
		TxConfigReg =AD7792_CONF_UNIPOLAR|AD7792_CONF_GAIN(AD7792_GAIN_1);
		AD7792_SetMode(AD7792_MODE_SEL(AD7792_MODE_CONT)|AD7792_MODE_CLKSRC(AD7792_CLK_INT)|/*AD7793_MODE_RATE(0x2)*/0xA);
		AD7792_SetIntReference(AD7792_REFSEL_EXT);
		AD7792_SetChannel(AD7792_CH_AIN1P_AIN1M);
		AD7792_SetRegisterValue(AD7792_REG_CONF, TxConfigReg, 2);// установка конфигурации
		AD7792_SetRegisterValue(AD7792_REG_IO, IO_Register, 1);	// установка генератора тока
	}
	else{
		TxConfigReg =AD7792_CONF_BIPOLAR|AD7792_CONF_GAIN(AD7792_GAIN_1);
		AD7792_SetMode(AD7792_MODE_SEL(AD7792_MODE_CONT)|AD7792_MODE_CLKSRC(AD7792_CLK_INT)|/*AD7792_MODE_RATE(0x2)*/0xA);
		AD7792_SetIntReference(AD7792_REFSEL_EXT);///AD7792_REFSEL_EXT
		AD7792_SetChannel(AD7792_CH_AIN1P_AIN1M);
		AD7792_SetRegisterValue(AD7792_REG_CONF, TxConfigReg, 2);
		AD7792_SetRegisterValue(AD7792_REG_IO, IO_Register, 1);				// установка генератора тока
	}
	status=0x01;
	return (status);
}

void Select_SPI_Device(uint8_t i)
{
	switch(i)
	{
		case 0:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);
		    break;
		case 1:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, RESET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);

			break;
		case 2:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, RESET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);
			break;
		case 3:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, RESET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);
			break;
		case 4:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, RESET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);
			break;
		case 5:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, RESET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);
			break;
		case 6:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, RESET);
			break;
		default:
			HAL_GPIO_WritePin(nCS1_GPIO_Port, nCS1_Pin, SET);
			HAL_GPIO_WritePin(nCS2_GPIO_Port, nCS2_Pin, SET);
			HAL_GPIO_WritePin(nCS3_GPIO_Port, nCS3_Pin, SET);
			HAL_GPIO_WritePin(nCS4_GPIO_Port, nCS4_Pin, SET);
			HAL_GPIO_WritePin(nCS5_GPIO_Port, nCS5_Pin, SET);
			HAL_GPIO_WritePin(nCS6_GPIO_Port, nCS6_Pin, SET);
			break;
	}
	HAL_Delay(10);

}

