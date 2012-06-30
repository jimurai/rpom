#ifndef __BSP_H
#define __BSP_H  

#ifdef __cplusplus
extern "C" {
#endif
              
#include <stdint.h>
#include <stdbool.h>
#include "efm32.h"   
#include "em_cmu.h"
#include "em_gpio.h"  
#include "em_usart.h"     
#include "em_dac.h"    
#include "em_adc.h"          
#include "em_opamp.h"       
#include "em_letimer.h"     
#include "em_timer.h"  

#define DAC_LDACn_SET()   (GPIO->P[2].DOUTSET = 1 << 13)
#define DAC_LDACn_CLR()   (GPIO->P[2].DOUTCLR = 1 << 13)
#define DAC_CLRn_SET()    (GPIO->P[2].DOUTSET = 1 << 14)
#define DAC_CLRn_CLR()    (GPIO->P[2].DOUTCLR = 1 << 14)    
#define DAC_CSn_SET()     (GPIO->P[4].DOUTSET = 1 << 13)
#define DAC_CSn_CLR()     (GPIO->P[4].DOUTCLR = 1 << 13)

#define LED0_ON()         (GPIO->P[1].DOUTSET = 1 << 11)
#define LED0_OFF()        (GPIO->P[1].DOUTCLR = 1 << 11)
#define LED1_ON()         (GPIO->P[1].DOUTSET = 1 << 13)
#define LED1_OFF()        (GPIO->P[1].DOUTCLR = 1 << 13)  
#define LEDS_OFF()        (GPIO->P[1].DOUTCLR = 3 << 11)

#define ITIA_RESET()      (GPIO->P[0].DOUTSET = 1 << 0)
#define ITIA_RELEASE()    (GPIO->P[0].DOUTCLR = 1 << 0)

void Delay(uint32_t dlyTicks);
void CMU_init(void);
void GPIO_init(void);
void LFTIMER_init(void);
void HFTIMER_init(void);      		      
void USART0_spi_init(void);
void USART1_uart_init(void);

void xDAC_init(void);
void xDAC_write(char command, uint16_t value, bool auto_sync);  

void iDAC_init(void);          
void iDAC_write(uint16_t value);       

void iADC_init(void);
        
#ifdef __cplusplus
}
#endif

#endif
