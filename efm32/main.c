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
uint8_t channel, adc_channel;
/* Homodyne detection systems */
#define FILTER_ORDER 5
HOMODYNE_CHANNEL channels[2];
int8_t osc_lut[4] = {-1, 0, 1, 0};
LWDF_ALPHA hd_alphas[FILTER_ORDER] = {1804,2847,25,1073,24};
uint8_t hd_types[FILTER_ORDER] = {0,3,0,3,0};


/* Comm's variables */
char_fifo_t tx_fifo;
char_fifo_t rx_fifo;

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Local variables */
  int32_t measurement,temp_i;
  // Note that the chip resets to using the HFRCO at 14MHz
  /* Chip errata */
  CHIP_Init();               
  /* If first word of user data page is non-zero, enable eA Profiler trace */
  TRACE_ProfilerSetup();               
  /* Initialise core functionality */
  CMU_init();
  GPIO_init();
  
  /* Set up the homodynde detection system */
  channel = 0;
  uint8_t i,j = 0;
  for (i=0;i<2;i++) {
    channels[i].local_osc.local_osc = osc_lut;
    channels[i].local_osc.phase_limit = 4;
    channels[i].local_osc.phase_offset = 0;
    channels[i].filter.order = FILTER_ORDER;
    for (j=0;j<FILTER_ORDER;j++) {
      channels[i].filter.alphas[j] = hd_alphas[j];
      channels[i].filter.types[j] = hd_types[j];
    }
  }
  // Make channel 0 orthogonal to channel 1 (pi/2 offset)
  channels[1].local_osc.phase_offset = 1;
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
  iDAC_write(0);
  iADC_init();

  // DAC_A
  uint8_t dac_command = (3 << 3);
  xDAC_write(dac_command, 0x100, true);          
  // DAC_B
  dac_command = (3 << 3) | (1 << 0);
  xDAC_write(dac_command, 0x108, true);

  /* Set up low frequency timer for sampling */
  LFTIMER_init();

  /* Set up high accuracy timer for precision timing procedures */
  HFTIMER_init();
  
  /* Infinite main loop */
  while (1) {
    for (temp_i=0;temp_i<2;temp_i++) {
      /* Test the first optical channel for waiting data */
      if (channels[temp_i].adc_waiting) {
        /* Perform homodyne detection on raw ADC value */
        measurement = hd_adc_process(&channels[temp_i]);
        /* Store the result for transmission */
        sample_message.payload[(temp_i<<1)]   = (uint8_t)(measurement>>18);
        sample_message.payload[(temp_i<<1)+1] = (uint8_t)((measurement>>10)&0x00FF);
        /* If all channels complete then transmit the packet */
        if (temp_i==1) {
          sample_message.header.length = sizeof(sample_message.header) + 4;
          tx_message(&sample_message, &tx_fifo);
        }
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
  uint8_t rxdata;
  /* Checking that RX-flag is set*/
  if (USART1->STATUS & USART_STATUS_RXDATAV) {
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
    adc_channel = channel;
    ADC_Start(ADC0, adcStartSingle);       
    ADC0->IEN = ADC_IEN_SINGLE;
    NVIC_EnableIRQ(ADC0_IRQn);

    /* Trigger conversion & reset timer */           
    TIMER_Enable(TIMER0,true);
    TIMER_IntEnable(TIMER0, TIMER_IF_OF);
    NVIC_EnableIRQ(TIMER0_IRQn);
    
    /* Increment the phase of the homodynde detection system */
    hd_increment_phase(&channels[channel]);

    /* Set VCCS current depending on next optical channel */
    if (channel==0) {
      LED0_OFF();              
      /* Prep for LED1(IR) channel */
      channel = 1;
      iDAC_write(300);
      /* Turn on LED immediate if in phase 2 */
      if (channels[1].led_phase==2) LED1_ON();
    }
    else {
      LED1_OFF();     
      /* Prep for LED0(RED) channel */
      channel = 0;
      iDAC_write(2000);
      /* Turn on LED immediate if in phase 2 */
      if (channels[0].led_phase==2) LED0_ON(); 

    }

    /* Clear interrupt flag manually */
    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
  }                                                     
  
  if (flags& LETIMER_IF_COMP1) {
    if ((channel==0) && (channels[0].led_phase&0x01))
      LED0_ON();
    else if ((channel==1) && (channels[1].led_phase&0x01))
      LED1_ON();
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
  /* Reset the intergrator now the conversion is complete */
  ITIA_RESET();
  /* Clear ADC0 interrupt flag */
  ADC0->IFC = 1;
  /* Read conversion result to clear Single Data Valid flag */
  uint32_t adcResult = ADC_DataSingleGet(ADC0);
  /* Store the value for subsequent processing */
  hd_adc_hold(&channels[adc_channel], (int16_t)(adcResult));
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
