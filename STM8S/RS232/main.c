/* Copyright (c) 2015 Secret Labs LLC. All Rights Reserved. */

#include "main.h"

#define PIN_UNUSED 0xFF

/* NOTE: UART RX buffer must be at least 2 bytes and no more than 256 bytes; one buffer position is reserved */
#define UART_RX_BUFFER_SIZE 32
/* NOTE: UART TX buffer must be at least 10 bytes and no more than 256 bytes; one buffer position is reserved */
#define UART_TX_BUFFER_SIZE 32

/* RX UART buffer */
//unsigned char _uartRxBuffer[UART_RX_BUFFER_SIZE];
//volatile unsigned char _uartRxBuffer_Begin = 0; /* this index is the first character in buffer; this index is updated when bytes are removed from buffer */
//volatile unsigned char _uartRxBuffer_End = 0; /* this index is one greater than the last character in buffer; this index is updated only by the "RX complete" handler */

/* TX UART buffer */
unsigned char _uartTxBuffer[UART_TX_BUFFER_SIZE];
volatile unsigned char _uartTxBuffer_Begin = 0;  /* this index is the first character in buffer; this index is updated when bytes are removed from buffer */
/* NOTE: (_uartTxBuffer_Begin == _uartTxBuffer_End) means that there is no data in the buffer. */
volatile unsigned char _uartTxBuffer_End = 0; /* this index is one greater than the last character in buffer; this index is updated only by the "add bytes to TX buffer" function */

/* UART RTS/CTS pins (optional; use PIN_UNUSED (0xFF) if not configured */
//UART_Type* _uart = UART1;
GPIO_Type* _uartRtsPort = GPIOC; /* use any GPIOx if not configured */
unsigned char _uartRtsPin = 2;
volatile bool _uartRtsEnabled = FALSE;
GPIO_Type* _uartCtsPort = GPIOB; /* use any GPIOx if not configured */
unsigned char _uartCtsPin = 0;
volatile bool _uartCtsEnabled = FALSE;

/* cache GPIO input values so that the GPIO interrupt routines know which pins have changed */
volatile unsigned char _gpiobLastInputValues = 0xFF;
volatile unsigned char _gpiocLastInputValues = 0xFF;

int main( void )
{
  /* defaults upon device power-up:
   * hsi is enabled and selected
   * mcu is set to hsi/8 (16,hz/2 = 2mhz)
   * hse is disabled
   * lsi is disabled
   * all peripherals are enabled */
  
  /* initialize our clock (8MHz = HSI 16MHz / 2), with fast wake-up from halt mode */
  MasterClock_Initialize();  

  /* turn off all peripheral clocks; we'll enable the ones we need after this. */
  STM8S_DisableAllPeripheralClocksAfterReset();
  
  /* initialize all of our module's unused GPIOs to input mode, pull-up enabled, interrupt disabled */
  GPIO_InitializeToSafeDefaults();

  /* configure our module-specific pins */
  GPIO_Configure();
 
  /* configure our module's non-GPIO peripherals */
  /* UART peripheral */
  UART_Initialize();
  
  /* enable our UART */
  UART_Enable();

  /* configure GoBus SPI slave transport */
  GoBus_Initialize();

  /* important: after initializing all peripherals and features, enable interrupts */
  __enable_interrupt();

  /* TEMPORARY: write up to UART_TX_BUFFER_SIZE of "i've started up" bytes. */
  UART_SendChar('H');
  UART_SendChar('i');
  UART_SendChar('!');
  
  /* send uart characters forever */
//  char index = 0;
//  char bytesWritten = 0;
//  while (TRUE)
//  {
//    bytesWritten = UART_SendChar(index);
//    index += bytesWritten;
//  }
  
  /* TEMP: do not exit! */
  while( TRUE );
}

void MasterClock_Initialize( void )
{
  /* configure master clock as: (HSE 8MHz / 1) */
  
  /* configure HSE as clock master */
  CLK->SWR = CKL_SWR_SWI_HSE;
  
  /* wait for the target clock source (HSE) to be ready; we are now for the switch */
  while (!(CLK->SWCR & CLK_SWCR_SWIF));
  
  /* clear the SWIF flag */
  CLK->SWCR &= ~CLK_SWCR_SWIF;

  /* switch master clock to HSE */
  CLK->SWCR |= CLK_SWCR_SWEN;

  /* wait for master clock switch to complete */
  while (CLK->SWCR & CLK_SWCR_SWBSY);
  
  /* stop switching the clock */
  CLK->SWCR &= ~CLK_SWCR_SWEN;
    
  /* disable HSI */
  CLK->ICKR &= ~CLK_ICKR_HSIEN;
}

void GPIO_InitializeToSafeDefaults( void )
{
  /* NOTE: all GPIOs are set to input mode with interrupt disabled by default, so we are only modifying pull-up resistor state here. */

  /* set all unused GPIOs to input mode, pull-up enabled, interrupt disabled by default */
  unsigned char GPIOA_CR1 = 0xFF;
  unsigned char GPIOB_CR1 = 0xFF;
  unsigned char GPIOC_CR1 = 0xFF;
  unsigned char GPIOD_CR1 = 0xFF;
  unsigned char GPIOE_CR1 = 0xFF;
  
  /* disable pull-ups on our shared GoBus SPI pins */
  /* PC5: SPI_SCK */
  GPIOC_CR1 &= ~(1 << 5); /* pull-up disabled */  
  /* PC6: SPI_MOSI */
  GPIOC_CR1 &= ~(1 << 6); /* pull-up disabled */  
  /* PC7: SPI_MISO */
  GPIOC_CR1 &= ~(1 << 7); /* pull-up disabled */  
  
  /* disable pull-ups on I2C pins (commented out because it's not necessary; I2C pins ignore these settings) */
  /* PB4: I2C_SCL */
  /* GPIOB_CR1 &= ~(1 << 4); */ /* pull-up disabled */
  /* PB5: I2C_SDA */
  /* GPIOB_CR1 &= ~(1 << 5); */ /* pull-up disabled */
  
  /* disable pull-ups on our module-specific pins */
  /* PA1: OSCIN */
  GPIOA_CR1 &= ~(1 << 1); /* pull-up disabled */  
  /* PA2: OSCOUT */
  GPIOA_CR1 &= ~(1 << 2); /* pull-up disabled */  
  /* PA3: LED */
  GPIOA_CR1 &= ~(1 << 3); /* pull-up disabled */  
  /* PB6: FORCEON */
  GPIOB_CR1 &= ~(1 << 6); /* pull-up disabled */  
  /* PB7: /FORCEOFF */
  GPIOB_CR1 &= ~(1 << 7); /* pull-up disabled */  
  /* PC1: /INVALID */
  GPIOC_CR1 &= ~(1 << 1); /* pull-up disabled */  
  /* PC2: UART_RTS */
  GPIOC_CR1 &= ~(1 << 2); /* pull-up disabled */  
  /* PC3: UART_DTR */
  GPIOC_CR1 &= ~(1 << 3); /* pull-up disabled */  
  /* PD5: UART_TX */
  GPIOD_CR1 &= ~(1 << 5); /* pull-up disabled */  
  /* PD6: UART_RX */
  GPIOD_CR1 &= ~(1 << 6); /* pull-up disabled */  
  
  // configure all MCU pins
  GPIOA->CR1 = GPIOA_CR1;
  GPIOB->CR1 = GPIOB_CR1;
  GPIOC->CR1 = GPIOC_CR1;
  GPIOD->CR1 = GPIOD_CR1;
  GPIOE->CR1 = GPIOE_CR1;
}

void GPIO_Configure( void )
{
  /* NOTE: because we use SPI_SS as a GPIO interrupt, only the SPI_SS pin may be interrupt-enabled on its port; no other pins should be interrupt-enabled on the same GPIO port. */
  
  /* configure our module-specific pins */
  /* PB0: UART_CTS */
  GPIOB->CR1 |= (1 << 0); /* pull-up enabled */
  /* PB1: UART_DCD */
  GPIOB->CR1 |= (1 << 1); /* pull-up enabled */
  /* PB2: UART_DSR */
  GPIOB->CR1 |= (1 << 2); /* pull-up enabled */
  /* PB3: UART_RI */
  GPIOB->CR1 |= (1 << 3); /* pull-up enabled */
  /* PC1: INVALID */ 
  /* GPIOC->CR1 &= ~(1 << 1); */ /* pull-up disabled */ /* not needed */
  /* PC2: UART_RTS */
  GPIOC->ODR &= ~(1 << 2); /* output value: low */
  GPIOC->DDR |= (1 << 2); /* direction: output */
  GPIOC->CR1 |= (1 << 2); /* output type: push-pull */  
  /* PC3: UART_DTR */
  GPIOC->ODR |= (1 << 3); /* output value: high */
  GPIOC->DDR |= (1 << 3); /* direction: output */
  GPIOC->CR1 |= (1 << 3); /* output type: push-pull */  
  /* PD5: UART_TX */
  GPIOD->ODR |= (1 << 5); /* output value: high */
  GPIOD->DDR |= (1 << 5); /* direction: output */
  GPIOD->CR1 |= (1 << 5); /* output type: push-pull */  

  /* PA3: LED */
  GPIOA->ODR &= ~(1 << 3); /* output value: low */
  GPIOA->DDR |= (1 << 3); /* direction: output */
  GPIOA->CR1 |= (1 << 3); /* output type: push-pull */  

  /* PB6: FORCEON */
  GPIOB->ODR &= ~(1 << 6); /* output value: low | auto power-down mode enable */
//  GPIOB->ODR |= (1 << 6); /* output value: high | auto power-down mode disable */
  GPIOB->DDR |= (1 << 6); /* direction: output */
  GPIOB->CR1 |= (1 << 6); /* output type: push-pull */  
  /* PB7: /FORCEOFF */
//  GPIOB->ODR &= ~(1 << 7); /* output value: low */
  GPIOB->ODR |= (1 << 7); /* output value: high */
  GPIOB->DDR |= (1 << 7); /* direction: output */
  GPIOB->CR1 |= (1 << 7); /* output type: push-pull */  

  /* enable rising/falling edges on interrupts */
  EXTI->CR1 |= (0x03 << 2); /* Port B interrupt on rising and falling edges */
  EXTI->CR1 |= (0x03 << 4); /* Port C interrupt on rising and falling edges */
//  EXTI->CR2 |= (0x03 << 0); /* Port E interrupt on rising and falling edges */ <----------- this is for gobus and should NOT be in this function
  
  /* cache GPIO input values so that the GPIO interrupt routines know which pins have changed */
  _gpiobLastInputValues = GPIOB->IDR;
  _gpiocLastInputValues = GPIOC->IDR;
}

/* GPIOB interrupt handler */
#pragma vector = EXT1_vector
__interrupt void GPIOB_IRQHandler( void )
{
  unsigned char gpioValues = GPIOB->IDR;

  /* TODO: check GPIOB pins */
  
  /* save our current pin values (for comparison next time our interrupt is raised */
  _gpiobLastInputValues = gpioValues;
}

/* GPIOC interrupt handler */
#pragma vector = EXT2_vector
__interrupt void GPIOC_IRQHandler( void )
{
  unsigned char gpioValues = GPIOC->IDR;

  /* TODO: check GPIOC pins */
  
  /* save our current pin values (for comparison next time our interrupt is raised */
  _gpiocLastInputValues = gpioValues;
}

void UART_Initialize( void )
{
  /* enable our UART peripheral clock */
  CLK->PCKENR1 |= CLK_PCKENR1_UART1;

  /* set baud rate to 115200 bps, with calculations based off of 8MHz HSE */
  /* 8000000 / 115200 == 69.444; 69 = 0x45; 8000000 / 69 = 115942.029; accuracy = +0.644% */
  UART1->BRR2 = 0x05; /* BRR2 must be written first */
  UART1->BRR1 = 0x04; /* BRR1 must be written last: a write to BRR1 will update the baud counters */
  /* UART1->CR1 = {not needed; defaults to 8,N,# i.e. eight data bits, no parity} */
  /* UART->CR3 = {not needed; defaults to #,#,1 i.e. one stop bit} */ 
  UART1->CR2 |= (1 << 5); /* receiver interrupt enable (RXNE) */
}

void UART_Enable( void )
{
  /* TODO: we should really only enable our transmitter/receiver _when_ serial device is detected */
  UART1->CR2 |= (1 << 3); /* enable transmitter */
  UART1->CR2 |= (1 << 2); /* enable receiver */

//  if (_uartCtsEnabled == TRUE)
//  {
//    _uartCtsPort->DDR &= ~(1 << _uartCtsPin); /* direction: input */
//    _uartCtsPort->CR1 &= ~(1 << _uartCtsPin); /* pull-up disabled */
//    _uartCtsPort->CR2 |= (1 << _uartCtsPin); /* enable interrupt */
//    /* TODO: enable wakeup from CTS interrupt */
//  }
}

/* UART_SendChar enqeueues a character into the TX buffer and then returns the number of bytes queued */
unsigned char UART_SendChar(unsigned char value)
{
  unsigned char uartEndOfTxBufferPlusOne = ((_uartTxBuffer_End + 1) % UART_TX_BUFFER_SIZE);
  /* if there is room to write this character into the buffer, do so now. */
  if (uartEndOfTxBufferPlusOne != _uartTxBuffer_Begin)
  {
    /* queue the transmit character and advance our "end buffer" index by one. */
    _uartTxBuffer[_uartTxBuffer_End] = value;
    _uartTxBuffer_End = (_uartTxBuffer_End + 1) % UART_TX_BUFFER_SIZE;
    /* ensure that our UART TX interrupt is enabled */
    UART1->CR2 |= UART1_CR2_TIEN;
    /* value was queued in the buffer; return a "bytesWritten" count of one. */
    return 1;
  }
  else
  {
    /* no room remaining in the buffer; return a "bytesWritten" count of zero. */
    return 0;
  }
}

/* UART1 TX interrupt handler */
#pragma vector = UART1_TX_vector
__interrupt void UART1_TX_IRQHandler( void )
{
  /* check to see if our TX buffer has waiting data */
  unsigned char uartEndOfTxBuffer = _uartTxBuffer_End;
  if (_uartTxBuffer_Begin != uartEndOfTxBuffer)
  {
    /* if we are currently allowed to transmit data, do so now. */
//    if ((_uartCtsEnabled == TRUE) && ((_uartCtsPort->IDR & (1 << _uartCtsPin)) != 0))
//    {
//      /* CTS is high: do not transmit. */
//      UART1->CR2 &= ~UART1_CR2_TIEN;
//      /* wire up CTS interrupt; we will re-enable transmission when CTS is driven low */
//      _uartCtsPort->CR2 |= (1 << _uartCtsPin);
//      /* TODO: enable wakeup from CTS interrupt */
//    }				
//    else
//    {
      UART1->DR = _uartTxBuffer[_uartTxBuffer_Begin];
      _uartTxBuffer_Begin = (_uartTxBuffer_Begin + 1) % UART_TX_BUFFER_SIZE;
//    }
  }
  else if (UART1->CR2 & UART1_CR2_TIEN)
  {
    UART1->CR2 &= ~UART1_CR2_TIEN;
  }
}
