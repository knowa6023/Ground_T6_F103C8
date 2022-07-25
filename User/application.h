/*
 * application.h
 *
 *  Created on: Feb 28, 2021
 *      Author: PC
 */
#include "main.h"
#ifndef APPLICATION_H_
#define APPLICATION_H_
void App_main();
void ledTimer_Callback(void);
void eAPP_Chip_Un_Select();
void eAPP_Chip_Select();



typedef struct{
int32_t mas[12];
} PT1000_t;

#endif /* APPLICATION_H_ */
