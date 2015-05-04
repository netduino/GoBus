/* Copyright (c) 2015 Secret Labs LLC. All Rights Reserved. */

#ifndef __MAIN_H
#define __MAIN_H

#include "..\common\gobus.h"
#include "..\common\stm8s003k3.h"

/* our function prototypes */
// mandatory functions
void MasterClock_Initialize( void );
void GPIO_InitializeToSafeDefaults( void );
void GPIO_Configure( void );
/* peripheral-specific functions (required if peripheral is used) */
void UART_Initialize( void );
void UART_Enable( void );
unsigned char UART_SendChar(unsigned char value);

#endif /* __MAIN_H */