/* Copyright (c) 2015 Secret Labs LLC. All Rights Reserved. */

#include "stm8s003k3.h"

void STM8S_DisableAllPeripheralClocksAfterReset( void )
{
  /* turn off all peripheral clocks */
  /* note: since peripherals are all disabled at reset, we do not need to disable them before turning off their clocks */
  /* note: if we want to disable all clocks at any other time, we need to disable the peripherals first. */
  CLK->PCKENR1 &= ~CLK_PCKENR1_TIM1;
  CLK->PCKENR1 &= ~CLK_PCKENR1_TIM2;
  CLK->PCKENR1 &= ~CLK_PCKENR1_TIM4;
  CLK->PCKENR1 &= ~CLK_PCKENR1_UART1;
  CLK->PCKENR1 &= ~CLK_PCKENR1_SPI;
  CLK->PCKENR1 &= ~CLK_PCKENR1_I2C;
  CLK->PCKENR2 &= ~CLK_PCKENR2_ADC;
  CLK->PCKENR2 &= ~CLK_PCKENR2_AWU;
}