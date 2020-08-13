/**
  ******************************************************************************
  * @file    LoadCell.c
  * @author  Ankit Patel
  * @version V1.0
  * @date    07-August-2020
  * @brief   This file contains all the functions definition
  *          to interface the HX711 (&Loadcell) with STM32F4
  ******************************************************************************/
#include "LoadCell.h"
#include "stm32f4xx_gpio.h"

int HX711_Read(HX711* data) {
	int count = 0;
	_Bool isNegative = 0;
	GPIO_WriteBit(data->PD_SCK_PinType, data->PD_SCK_PinNumber, RESET);
	while (GPIO_ReadInputDataBit(data->DOUT_PinType, data->DOUT_PinNumber)) {  //monitor if DOUT goes low, data is ready!!

	}
	for (int i = 0; i < 24; i++) {                                    // send 24 pulses to read data from HX711 IC
		GPIO_WriteBit(data->PD_SCK_PinType, data->PD_SCK_PinNumber, SET);
		count = count << 1;
		GPIO_WriteBit(data->PD_SCK_PinType, data->PD_SCK_PinNumber, RESET);
		if (GPIO_ReadInputDataBit(data->DOUT_PinType, data->DOUT_PinNumber)) {
			if (i == 0) {
				isNegative = 1;
			}
			count++;
		}
	}
	if (isNegative) {
		count = count ^ 0xFF000000;
	}
	GPIO_WriteBit(data->PD_SCK_PinType, data->PD_SCK_PinNumber, SET);         // 25th Pulse for mode 0
	GPIO_WriteBit(data->PD_SCK_PinType, data->PD_SCK_PinNumber, RESET);
	for (int i = 0; i < data->gain; i++) {                                   //if gain is other than 0 this block executes
		GPIO_WriteBit(data->PD_SCK_PinType, data->PD_SCK_PinNumber, SET);    // if i=1 , channel= B,gain=32
		GPIO_WriteBit(data->PD_SCK_PinType, data->PD_SCK_PinNumber, RESET);  //if  i=2 , channel= A,gain=64
	}
	return count;
}

int HX711_AvgRead(HX711* data, int times) {
	int sum = 0;
	for (int i = 0; i < times; i++) {
		sum += HX711_Read(data);
	}
	return sum / times;
}

void HX711_Tare(HX711* data, int times) {
	data->offset = HX711_AvgRead(data, times);
}

int HX711_GetValue(HX711* data) {
	return HX711_Read(data) - (data->offset);
}

int HX711_GetAvgValue(HX711* data, int times) {
	return HX711_AvgRead(data, times) - (data->offset);
}
