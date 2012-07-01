 /**
 * @file
 * @brief Reflective Pulse Oximeter Module
 *
 * @author James A. C. Patterson
 * @version 0.1
 * @date 31/05/2012
 *
 * @section LICENSE
 *
 * Free Beer License
 *
 */
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

#include "homodyne.h"
#include "ppg_delineator.h"

/* State variables */
uint8_t phase;
uint8_t channel;
uint8_t mode;
struct sample_s {
  bool      waiting;
  uint8_t   channel;
  uint16_t  value;
} sample;
HOMODYNE_CHANNEL channels[2];

/* Comm's variables */
char_fifo_t tx_fifo;
char_fifo_t rx_fifo;

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Local variables */
  int16_t measurement;
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
  
  /* Set up the homodynde detection system */
  channels[0].filter.order = 5;
  channels[1].filter.order = 5;
  hd_reset(&channels[0]);
  hd_reset(&channels[1]);
  
  
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
  while (1) {
    /* Test the first optical channel for waiting data */
    if (channels[0].adc_waiting) {
      measurement = hd_adc_process(&channels[0]);
      sample_message.payload[0] = (uint8_t)(measurement>>8);
      sample_message.payload[1] = (uint8_t)(measurement&0x00FF);
    }
    /* Test the second optical channel for waiting data */
    if (channels[1].adc_waiting) {
      measurement = hd_adc_process(&channels[1]);
      sample_message.payload[2] = (uint8_t)(measurement>>8);
      sample_message.payload[3] = (uint8_t)(measurement&0x00FF);
      sample_message.header.length = sizeof(sample_message.header) + 4;
      tx_message(&sample_message, &tx_fifo);
    }
//    if (sample.waiting) {
//      sample.waiting = false;
//      if (sample.channel == 1) {
//        sample_message.payload[0] = (uint8_t)(sample.value>>8);
//        sample_message.payload[1] = (uint8_t)(sample.value&0x00FF);
//      }
//      else {
//        sample_message.payload[2] = (uint8_t)(sample.value>>8);
//        sample_message.payload[3] = (uint8_t)(sample.value&0x00FF);
//        sample_message.header.length = sizeof(sample_message.header) + 4;
//        tx_message(&sample_message, &tx_fifo);
//      }
//    }
  }
}          
                                   
/**************************************************************************//**
 * @brief USART1 IRQ Handler
 * set up the interrupt prior to use
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  uint8_t rxdata;
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
    
    /* Increment the phase of the homodynde detection system */
    hd_increment_phase(&channels[channel]);

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
      iDAC_write((uint32_t)(4000));
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
  /* Store the value for subsequent processing */
  hd_adc_hold(&channels[channel], (int16_t)(adcResult))  ;
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
