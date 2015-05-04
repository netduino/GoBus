/* Copyright (c) 2015 Secret Labs LLC. All Rights Reserved. */

#include "gobus.h"

/* critical functions are used to disable interrupts during a critical section of code. */
/* NOTE: these functions must NOT be called from interrupts */
/* NOTE: because MONITOR_ENTER calls CRITICAL_ENTER/EXIT, it must never be called inside of a critical section or the interrupt state may be corrupted and the critical state will be exited prematurely */
__istate_t _interruptState;
#define CRITICAL_ENTER()   _interruptState = __get_interrupt_state(); __disable_interrupt()
#define CRITICAL_EXIT()    __set_interrupt_state(_interruptState)
#define MONITOR_ENTER( monitorToken ) while(TRUE) {while (monitorToken); CRITICAL_ENTER(); if (!monitorToken) { monitorToken = TRUE; CRITICAL_EXIT(); break; } else { CRITICAL_EXIT(); } }
#define MONITOR_EXIT( monitorToken )  monitorToken = FALSE

/* frame buffers (do not allocate final byte, as final byte is generated/validated by STM8S */
#define FRAME_SIZE 18

/* rx frame buffer (currently buffered during reception */
unsigned char _rxFrameBuffer[FRAME_SIZE - 1];
unsigned char _rxFrameBufferIndex = 0;
/* our queue and protocol state flags */
//volatile bool _lastRxFrameInvalid = FALSE;

/* init our GoBus SPI slave transport */
bool GoBus_Initialize( void )
{
  /* enable SPI peripheral clock */
  CLK->PCKENR1 |= CLK_PCKENR1_SPI;
  
//  /* enable SPI CRC8 before the SPI peripheral is enabled */
//  SPI->CR2 |= SPI_CR2_CRCEN;
  /* NOTE: we tesetd both software and hardware SPI selection and determined that hardware selection is better
   *       we also determined that hardware selection returns MISO to "input, floating, no interrupt" after SPI_SS is
   *       de-asserted (and presumably configures the push-pull mode and speed correctly when in use).
   * NOTE: when a frame is received corrupted, we will clear the current frame and manually handle SPI_SS until it is deasserted */
//  /* enable SPI SS software management to ensure that our frames are aligned to SPI_SS assertions...and deassert software select */
//  SPI->CR2 |= SPI_CR2_SSM | SPI_CR2_SSI;

  /* set capture to falling edge */
  SPI->CR1 |= SPI_CR1_CPHA;
  
  /* configure SPI interrupts; TXIE will be enabled once we write a byte */
  SPI->ICR &= ~SPI_ICR_TXIE;
  SPI->ICR |= SPI_ICR_RXIE; /* | SPI_ICR_ERRIE; */

  /* set the transmit buffer to an empty frame */
  SPI->DR = 0x00;
  /* _txFrameBufferIsEmpty = TRUE; */ /* NOT NEEDED: DEFAULT VALUE */
  
  /* configure SPI pins as:
   * SPI_CLK, SPI_MOSI, SPI_SS: input + floating 
   * SPI_MISO: push-pull + fast slope */
  /* NOTE: according to STM8 Reference Manual 20.3.2 note, SPI_MISO must be configured as input mode, pull-up enabled, no interrupt */
  
  /* configure our GOBUS_SS pin */
  /* GOBUS_SS */
  GPIO_GOBUS_SS_PORT->CR1 |= (1 << GPIO_GOBUS_SS_PIN); /* pull-up enabled */
//  GPIO_GOBUS_SS_PORT->CR2 |= (1 << GPIO_GOBUS_SS_PIN); /* interrupt enabled */
  /* enable our SPI_SS pin GPIO external interrupt; normally we let the SPI peripheral handle its own SPI_SS pin, but we can use this interrupt as necessary to resync SPI frame boundaries */
#if defined(EXTI_GOBUS_SS_PORT_0)
  EXTI->CR1 |= (0x03 << 0); /* Port A interrupt on rising and falling edges */
#elif defined(EXTI_GOBUS_SS_PORT_4)
  EXTI->CR2 |= (0x03 << 0); /* Port E interrupt on rising and falling edges */
#endif

  /* disable pull-ups on our GoBus SPI pins */
  /* GOBUS_SCK */
  GPIO_GOBUS_SCK_PORT->CR1 &= ~(1 << GPIO_GOBUS_SCK_PIN); /* pull-up disabled */
  /* GOBUS_MOSI */
  GPIO_GOBUS_MOSI_PORT->CR1 &= ~(1 << GPIO_GOBUS_MOSI_PIN); /* pull-up disabled */
  /* GOBUS_MISO */ 
  GPIO_GOBUS_MISO_PORT->CR1 &= ~(1 << GPIO_GOBUS_MISO_PIN); /* pull-up disabled */

  /* enable SPI */
  SPI->CR1 |= SPI_CR1_SPE;  

  /* note: we do not __enable_interrupt() here; our main code is responsible for enabling/disabling interrupts system-wide and will enable them after all initialization is complete */
  /* note: our caller must enable our transport once it has fully booted, so that we can assert /INT to notify our master that we exist. */
  
//  /* queue up a full reset frame; when our /INT pin is activated this will be ready for transmission to the GoBus master */
//  GoBus_SendResetFrame( TRUE );
//  /* now enable TXIE interrupts */
//  SPI->ICR |= SPI_ICR_TXIE;
  return TRUE;
}

/* SPI interrupt handler */
#pragma vector = SPI_vector
 __interrupt void SPI_IRQHandler( void )
{
//  volatile unsigned char temp;
  if(SPI->SR & SPI_SR_RXNE)
  {
    /* SPI 1-byte (register) receive buffer is filled */
    if (_rxFrameBufferIndex < FRAME_SIZE - 1)
    {
      _rxFrameBuffer[_rxFrameBufferIndex++] = SPI->DR;
    }
  }
}
