/* Copyright (c) 2015 Secret Labs LLC. All Rights Reserved. */

#ifndef __GLOBALDEFS_H
#define __GLOBALDEFS_H

#include "..\common\stm8s003k3.h"

/* our pins */
///* /GOBUS_INT */
//#define GPIO_GOBUS_INT_PORT GPIOD
//#define GPIO_GOBUS_INT_PIN 7
/* /GOBUS_SS */ /* NOTE: no other interrupts may exist on the same PORT as the SPI_SS pin, and the SPI_SS pin must be hardware SPI_SS capable */
#define GPIO_GOBUS_SS_PORT GPIOE
#define GPIO_GOBUS_SS_PIN 5
//#define EXTI_GOBUS_SS_PORT_0 /* PORT 0 == GPIOA */
#define EXTI_GOBUS_SS_PORT_4 /* PORT 4 == GPIOE */
/* GOBUS_SCK */
#define GPIO_GOBUS_SCK_PORT GPIOC
#define GPIO_GOBUS_SCK_PIN 5
/* GOBUS_MOSI */
#define GPIO_GOBUS_MOSI_PORT GPIOC
#define GPIO_GOBUS_MOSI_PIN 6
/* GOBUS_MISO */
#define GPIO_GOBUS_MISO_PORT GPIOC
#define GPIO_GOBUS_MISO_PIN 7

#endif /* __GLOBALDEFS_H */