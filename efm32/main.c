/**************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo for EFM32TG_STK3300
 * @author Energy Micro AS
 * @version 3.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"  

#include "em_cmu.h"     
#include "em_gpio.h" 
                        
#include "trace.h"
#include "bsp.h"
#include "comms.h"  

/* State variables */
uint8_t phase;
uint8_t channel;
uint8_t mode;
struct sample_s {
  bool      waiting;
  uint8_t   channel;
  uint16_t  value;
} sample;

/* Comm's variables */
char_fifo_t tx_fifo;
char_fifo_t rx_fifo;

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{                              
  // Note that the chip resets to using the HFRCO at 14MHz
  /* Chip errata */
  CHIP_Init();               
  /* If first word of user data page is non-zero, enable eA Profiler trace */
  TRACE_ProfilerSetup();               
  /* Initialise core functionality */
  CMU_init();
  GPIO_init();

  /* Set up low frequency timer for sampling */
  LFTIMER_init();

  /* Set up high accuracy timer for precision timing procedures */
  HFTIMER_init();
  
  /* Set phase and state variables */
  phase = 0;
  channel = 0;
  mode = 0;
  sample.waiting = false;
  
  /* Manually initialise TX FIFO */
  tx_fifo.depth = 0;
  tx_fifo.pRead = 0;
  tx_fifo.pWrite = 0;
  
  /* Create a message */
  message_t sample_message;
  sample_message.header.id = 0x00;
  sample_message.header.type = 0x00;
  sample_message.header.length = 0;
  sample_message.header.check = 0;
  
  /* Set up functional peripheral interfaces */
  USART1_uart_init();
  iDAC_init();
  xDAC_init();  
  iDAC_write((uint32_t)((0.0 * 4096) / 1.25));
  iADC_init();

  // DAC_A
  uint8_t dac_command = (3 << 3);
  xDAC_write(dac_command, 0x100, true);          
  // DAC_B
  dac_command = (3 << 3) | (1 << 0);
  xDAC_write(dac_command, 0x108, true);
  
  /* Infinite main loop */
  static uint8_t count = 0;
  while (1) {
    if (sample.waiting) {
      sample.waiting = false;
      if (sample.channel == 1) {
        sample_message.payload[0] = (uint8_t)(sample.value>>8);
        sample_message.payload[1] = (uint8_t)(sample.value&0x00FF);
      }
      else {
        sample_message.payload[2] = (uint8_t)(sample.value>>8);
        sample_message.payload[3] = (uint8_t)(sample.value&0x00FF);
        sample_message.header.length = sizeof(sample_message.header) + 4;
        tx_message(&sample_message, &tx_fifo);
      }
    }
  }
}          
                                   
/**************************************************************************//**
 * @brief USART1 IRQ Handler
 * set up the interrupt prior to use
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  static uint8_t rxdata;
  /* Checking that RX-flag is set*/
  if (USART1->STATUS & USART_STATUS_RXDATAV) {
    mode = ~mode;
    rxdata = USART1->RXDATA;        
    /* Checking that the USART is waiting for data */     
//    while (!(USART1->STATUS & USART_STATUS_TXBL)) ;
    /* Transmitting the next byte */
//    USART1->TXDATA = rxdata;                              
    /*Waiting for transmission of last byte */
//    while (!(USART1->STATUS & USART_STATUS_TXC));
  }
}  

/**************************************************************************//**
 * @brief USART1 TX IRQ Handler
 * Sending until all data has been sent.
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  /* Check that the USART is waiting for data to TX */
  if (USART1->STATUS & USART_STATUS_TXBL)
  {
    /* Increment read pointer. Wrap if required. */
    if (++tx_fifo.pRead>=UART_BUFFER_DEPTH) tx_fifo.pRead=0;
    tx_fifo.depth--;                             
    /* Disable the interrupt when the FIFO is empty */
    if (tx_fifo.depth == 0) {
      USART1->IEN &= ~USART_IEN_TXBL;
      NVIC_DisableIRQ(USART1_TX_IRQn);
      NVIC_ClearPendingIRQ(USART1_TX_IRQn);
    }
    /* Otherwise transmit the next byte in the FIFO */
    else {
      USART1->TXDATA = tx_fifo.buffer[tx_fifo.pRead];
    }
  }
}  
                             
/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{ 
  uint32_t flags = LETIMER_IntGet(LETIMER0);
  
  if (flags& LETIMER_IF_UF) {

    /* Trigger ADC conversion and enable interrupt*/    
    ADC_Start(ADC0, adcStartSingle);       
    ADC0->IEN = ADC_IEN_SINGLE;
    NVIC_EnableIRQ(ADC0_IRQn);

    /* Trigger conversion & reset timer */           
    TIMER_Enable(TIMER0,true);
    TIMER_IntEnable(TIMER0, TIMER_IF_OF);
    NVIC_EnableIRQ(TIMER0_IRQn);
    /* Reset the intergrator now the conversion is complete */
    ITIA_RESET();

    /* Set VCCS current depending on next optical channel */
    if (channel==0) {
      LED0_OFF();              
      /* Prep for IR channel */
      channel = 1;
      iDAC_write((uint32_t)((0.2 * 4096) / 1.25));
    }
    else {
      LED1_OFF();     
      /* Prep for RED channel */
      channel = 0;
      iDAC_write((uint32_t)(4095));
      /* Increment the phase at underflow. Reset if required. */
      if (++phase == 4) phase = 0;   

    }                                 

    /* Turn relevant LED ON immediately in phase 2 */
    if (phase == 2) {
      //if (channel==0) LED0_OFF();
      if (channel==0) LED0_ON();
      else            LED1_ON(); 
    }          

    /* Clear interrupt flag manually */
    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
  }                                                     
  
  if (flags& LETIMER_IF_COMP1) {
    /* Turn ON relevant LED after given period. Odd phases only. */
    if (phase&0x01) {
      //if (channel==0) LED0_OFF();
      if (channel==0) LED0_ON();
      else            LED1_ON();  
    }                                    
    /* Clear interrupt flag manually */
    LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
  }   

}                      

/**************************************************************************//**
 * @brief ADC0_IRQHandler
 * Interrupt Service Routine for ADC
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  /* Clear ADC0 interrupt flag */
  ADC0->IFC = 1;
  /* Read conversion result to clear Single Data Valid flag */
  uint32_t adcResult = ADC_DataSingleGet(ADC0);  
  /* Save value and flag that it is waiting to be processed */
  sample.waiting = true;
  sample.value = (uint16_t)(adcResult & 0xFFFF);
  if (channel == 0)
    sample.channel = 1;
  else 
    sample.channel = 0;
  /* Disable interrupt */
  ADC0->IEN &= ~(ADC_IEN_SINGLE);
  NVIC_DisableIRQ(ADC0_IRQn);
  /* Powerdown ADC? */
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  /* Release Intergrator from reset to begin integration cycle */
  ITIA_RELEASE();                   
  /* Disable interrupt */
  TIMER_IntDisable(TIMER0, TIMER_IF_OF);
  NVIC_DisableIRQ(TIMER0_IRQn);
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);             
  /* Put timer to bed */
  TIMER_Enable(TIMER0,false);
  TIMER_CounterSet(TIMER0,0);
  
}
