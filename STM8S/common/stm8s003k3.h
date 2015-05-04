/* Copyright (c) 2015 Secret Labs LLC. All Rights Reserved. */

#ifndef __STM8S003K3_H
#define __STM8S003K3_H

#include <intrinsics.h>

/* custom types */
typedef enum 
{
  FALSE = 0, 
  TRUE = !FALSE
} bool;

/* CLK peripheral - Clock control */
typedef struct
{
  volatile unsigned char ICKR;
  volatile unsigned char ECKR;
  unsigned char RESERVED1;
  volatile unsigned char CMSR;
  volatile unsigned char SWR;
  volatile unsigned char SWCR;
  volatile unsigned char CKDIVR;
  volatile unsigned char PCKENR1;
  volatile unsigned char CSSR;
  volatile unsigned char CCOR;
  volatile unsigned char PCKENR2;
  unsigned char RESERVED2;
  volatile unsigned char HSITRIMR;
  volatile unsigned char SWIMCCR;
} CLK_Type;

//typedef enum
//{
//  CLK_PRESCALER_HSIDIV2 = (unsigned char)0x08
//} CLK_Prescaler_Type;

//#define CLK_ICKR_FHWU     (unsigned char)0x04
//#define CLK_ICKR_HSIRDY   (unsigned char)0x02
#define CLK_ICKR_HSIEN    (unsigned char)0x01

#define CKL_SWR_SWI_HSE   (unsigned char)0xB4

#define CLK_SWCR_SWBSY    (unsigned char)0x01
#define CLK_SWCR_SWEN     (unsigned char)0x02
#define CLK_SWCR_SWIF     (unsigned char)0x08

//#define CLK_CKDIVR_HSIDIV (unsigned char)0x18

#define CLK_PCKENR1_TIM1  (unsigned char)0x80
#define CLK_PCKENR1_TIM2  (unsigned char)0x20
#define CLK_PCKENR1_TIM4  (unsigned char)0x10
#define CLK_PCKENR1_UART1 (unsigned char)0x08
#define CLK_PCKENR1_SPI   (unsigned char)0x02
#define CLK_PCKENR1_I2C   (unsigned char)0x01

#define CLK_PCKENR2_ADC   (unsigned char)0x08
#define CLK_PCKENR2_AWU   (unsigned char)0x04

typedef struct
{
  volatile unsigned char CR1;
  volatile unsigned char CR2;
} EXTI_Type;

/* FLASH peripheral */
//typedef struct
//{
//  volatile unsigned char CR1;
//  volatile unsigned char CR2;
//  volatile unsigned char NCR2;
//  volatile unsigned char FPR;
//  volatile unsigned char NFPR;
//  volatile unsigned char IAPSR;
//  unsigned char RESERVED1;
//  unsigned char RESERVED2;
//  volatile unsigned char PUKR;
//  unsigned char RESERVED3;
//  volatile unsigned char DUKR;
//} FLASH_Type;

//#define FLASH_CR1_HALT (unsigned char)0x08

/* GPIO peripheral */
typedef struct
{
  volatile unsigned char ODR;
  volatile unsigned char IDR;
  volatile unsigned char DDR;
  volatile unsigned char CR1;
  volatile unsigned char CR2;
} GPIO_Type;

/* SPI peripheral - Serial peripheral interface */
typedef struct
{
  volatile unsigned char CR1;
  volatile unsigned char CR2;
  volatile unsigned char ICR;
  volatile unsigned char SR;
  volatile unsigned char DR;
  volatile unsigned char CRCPR;
  volatile unsigned char RXCRCR;
  volatile unsigned char TXCRCR;
} SPI_Type;

#define SPI_CR1_SPE   (unsigned char)0x40
#define SPI_CR1_CPHA  (unsigned char)0x01

//#define SPI_CR2_CRCEN (unsigned char)0x20
//#define SPI_CR2_CRCNEXT (unsigned char)0x10
//#define SPI_CR2_SSM (unsigned char)0x02
//#define SPI_CR2_SSI (unsigned char)0x01

#define SPI_ICR_TXIE  (unsigned char)0x80
#define SPI_ICR_RXIE  (unsigned char)0x40
//#define SPI_ICR_ERRIE (unsigned char)0x20
//#define SPI_ICR_WKIE  (unsigned char)0x10

//#define SPI_SR_BSY    (unsigned char)0x80
//#define SPI_SR_OVR    (unsigned char)0x40
///* #define SPI_SR_MODF   (unsigned char)0x20 */
//#define SPI_SR_CRCERR (unsigned char)0x10
//#define SPI_SR_WKUP   (unsigned char)0x08
//#define SPI_SR_TXE    (unsigned char)0x02
#define SPI_SR_RXNE   (unsigned char)0x01

/* UART peripheral - Universal asynchronous receiver transmitter */
typedef struct
{
  volatile unsigned char SR;
  volatile unsigned char DR;
  volatile unsigned char BRR1;
  volatile unsigned char BRR2;
  volatile unsigned char CR1;
  volatile unsigned char CR2;
  volatile unsigned char CR3;
  volatile unsigned char CR4;
  volatile unsigned char CR5;
  volatile unsigned char GTR;
  volatile unsigned char PSCR;
} UART_Type;

#define UART1_CR2_TIEN  (unsigned char)(1 << 7)

/* Window watchdog peripheral */
//typedef struct
//{
//  volatile unsigned char CR;
//  volatile unsigned char WR;
//} WWDG_Type;

//#define WWDG_CR_WDGA (unsigned char)0x80
//#define WWDG_CR_T6   (unsigned char)0x40

/* peripheral base addresses */
#define GPIOA_BaseAddress (unsigned short)0x5000
#define GPIOB_BaseAddress (unsigned short)0x5005
#define GPIOC_BaseAddress (unsigned short)0x500A
#define GPIOD_BaseAddress (unsigned short)0x500F
#define GPIOE_BaseAddress (unsigned short)0x5014
//#define FLASH_BaseAddress (unsigned short)0x505A
#define EXTI_BaseAddress  (unsigned short)0x50A0
#define CLK_BaseAddress   (unsigned short)0x50C0
//#define WWDG_BaseAddress  (unsigned short)0x50D1
#define SPI_BaseAddress   (unsigned short)0x5200
#define UART1_BaseAddress (unsigned short)0x5230

/* peripheral declarations */
#define CLK ((CLK_Type *) CLK_BaseAddress)
#define EXTI ((EXTI_Type *) EXTI_BaseAddress)
//#define FLASH ((FLASH_Type *) FLASH_BaseAddress)
#define GPIOA ((GPIO_Type *) GPIOA_BaseAddress)
#define GPIOB ((GPIO_Type *) GPIOB_BaseAddress)
#define GPIOC ((GPIO_Type *) GPIOC_BaseAddress)
#define GPIOD ((GPIO_Type *) GPIOD_BaseAddress)
#define GPIOE ((GPIO_Type *) GPIOE_BaseAddress)
#define GPIOF ((GPIO_Type *) GPIOF_BaseAddress)
#define SPI ((SPI_Type *) SPI_BaseAddress)
#define UART1 ((UART_Type *) UART1_BaseAddress)
//#define WWDG ((WWDG_Type *) WWDG_BaseAddress)

/* interrupt vectors */
/* NOTE: all interrupt vectors have 2 added to them, to offset them beyond the RESET and TRAP vectors. */
#define VECTOR_OFFSET 2
//#define RESET_vector      0
//#define TRAP_vector       1
//#define TLI_vector        ( 0 + VECTOR_OFFSET)
//#define AWU_vector        ( 1 + VECTOR_OFFSET)
//#define CLK_vector        ( 2 + VECTOR_OFFSET)
//#define EXT0_vector       ( 3 + VECTOR_OFFSET)
#define EXT1_vector       ( 4 + VECTOR_OFFSET)
#define EXT2_vector       ( 5 + VECTOR_OFFSET)
//#define EXT3_vector       ( 6 + VECTOR_OFFSET)
//#define EXT4_vector       ( 7 + VECTOR_OFFSET)
#define SPI_vector        (10 + VECTOR_OFFSET)
//#define TIM1_UOUTB_vector (11 + VECTOR_OFFSET)
//#define TIM1_CC_vector    (12 + VECTOR_OFFSET)
//#define TIM2_UO_vector    (13 + VECTOR_OFFSET)
//#define TIM2_CC_vector    (14 + VECTOR_OFFSET)
#define UART1_TX_vector   (17 + VECTOR_OFFSET)
//#define UART1_RX_vector   (18 + VECTOR_OFFSET)
//#define I2C_vector        (19 + VECTOR_OFFSET)
//#define ADC1_vector       (22 + VECTOR_OFFSET)
//#define TIM4_vector       (23 + VECTOR_OFFSET)
//#define FLASH_vector      (24 + VECTOR_OFFSET)

/* our method prototypes */
void STM8S_DisableAllPeripheralClocksAfterReset( void );

#endif /* __STM8S003K3_H */