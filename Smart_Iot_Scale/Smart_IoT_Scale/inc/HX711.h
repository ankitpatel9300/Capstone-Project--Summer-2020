

#ifndef HX711_H_
#define HX711_H_

#include "board.h"
#include "chip.h"

#define HX711_GPIO LPC_GPIO2
#define HX711_AHB1ENR RCC_AHB1ENR_GPIOFEN

typedef struct _hx711
{
	LPC_GPIO_T *gpioSck ;
	LPC_GPIO_T *gpioData ;
	uint16_t pinsck ;
	uint16_t pinData ;
	int offset ;
	int gain ;

} HX711 ;

void HX711_Init(HX711 data);
HX711 HX711_Tare(HX711 data, uint8_t times);
int HX711_Value(HX711 data);
int HX711_AverageValue(HX711 data, uint8_t times);

#endif /* HX711_H_ */
