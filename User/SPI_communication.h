/*
 * SPI_communication.h
 *
 *  Created on: Mar 1, 2021
 *      Author: PC
 */

#ifndef SPI_COMMUNICATION_H_
#define SPI_COMMUNICATION_H_

/******************************************************************************/
#define ADI_PART_CS_LOW 	eAPP_Chip_Select();
#define ADI_PART_CS_HIGH	eAPP_Chip_Un_Select();
#define AD7792_RDY_STATE    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6 )
/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Writes data to SPI. */
unsigned char SPI_Write(unsigned char* data,
                        unsigned char bytesNumber);
/* Reads data from SPI. */
unsigned char SPI_Read(unsigned char* data,
                       unsigned char bytesNumber);



#endif /* SPI_COMMUNICATION_H_ */
