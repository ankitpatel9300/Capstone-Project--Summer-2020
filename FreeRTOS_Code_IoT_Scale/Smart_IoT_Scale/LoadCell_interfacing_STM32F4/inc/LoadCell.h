/**
  ******************************************************************************
  * @file    LoadCell.h
  * @author  Ankit Patel
  * @version V1.0
  * @date    07-August-2020
  * @brief   This file contains all the functions prototypes
  *          to interface the HX711 (&Loadcell) with STM32F4
  ******************************************************************************/


#ifndef LOADCELL_H_
#define LOADCELL_H_

#include "stm32f4xx.h"

typedef struct {
	GPIO_TypeDef* PD_SCK_PinType;	//GPIOx
	uint16_t PD_SCK_PinNumber;		//GPIO_Pin
	GPIO_TypeDef* DOUT_PinType;		//GPIOx
	uint16_t DOUT_PinNumber;		//GPIO_Pin
	int offset;
	int gain;		// 0 : Input channel A, gain=128
					// 1 : Input channel B, gain=32
					// 2 : Input channel A, gain=64
} HX711;

int HX711_Read(HX711* data);
int HX711_AvgRead(HX711* data, int times);
void HX711_Tare(HX711* data, int times);
int HX711_GetValue(HX711* data);
int HX711_GetAvgValue(HX711* data, int times);

#endif /* LOADCELL_H_ */
