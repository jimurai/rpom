#include "bsp.h"

volatile uint32_t msTicks; /* counts 1ms timeTicks */
/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
}  

/* Start up the default clocks so nothing else has to do it */
void CMU_init(void)
{                       
  /* DEFAULT: Using HFRCO at 14MHz as high frequency clock, HFCLK */
  
  /* LFXO setup */                            
  CMU->CTRL    = CMU->CTRL & (~_CMU_CTRL_LFXOBOOST_MASK) | CMU_CTRL_LFXOBOOST_70PCENT;
          
  /* Enable LE clock and LFXO oscillator */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
  CMU->OSCENCMD |= CMU_OSCENCMD_LFXOEN;
  /* Wait until LFXO ready */
  /* Note that this could be done more energy friendly with an interrupt in EM1 */
  while (!(CMU->STATUS & CMU_STATUS_LFXORDY)) ;      
    
  /* Select LFXO as clock source for LFACLK */
  CMU->LFCLKSEL = (CMU->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFXO;
  
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;  
}
                           
/* Initialise GPIO ports */
void GPIO_init(void)      
{                     
  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
                                          
  /* Pin PA0 is configured to Push-pull */
  GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_PUSHPULL;
  /* Pin PA1 is configured to Input disabled with pull-up */
  GPIO->P[0].DOUT |= (1 << 1);
  GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | GPIO_P_MODEL_MODE1_DISABLED;
  /* Pin PA2 is configured to Input disabled with pull-up */
  GPIO->P[0].DOUT |= (1 << 2);
  GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_DISABLED;
  /* Pin PB11 is configured to Push-pull */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE11_MASK) | GPIO_P_MODEH_MODE11_PUSHPULL;
  /* Pin PB13 is configured to Push-pull */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | GPIO_P_MODEH_MODE13_PUSHPULL;
  /* Pin PB14 is configured to Input disabled with pull-up */
  GPIO->P[1].DOUT |= (1 << 14);
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE14_MASK) | GPIO_P_MODEH_MODE14_DISABLED;
  /* To avoid false start, configure output US1_TX as high on PC0 */
  GPIO->P[2].DOUT |= (1 << 0);
  /* Pin PC0 is configured to Push-pull */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_PUSHPULL;
  /* Pin PC1 is configured to Input enabled */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | GPIO_P_MODEL_MODE1_INPUT;
  /* Pin PC13 is configured to Push-pull */
  GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | GPIO_P_MODEH_MODE13_PUSHPULL;
  /* Pin PC14 is configured to Push-pull */
  GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE14_MASK) | GPIO_P_MODEH_MODE14_PUSHPULL;   
  /* Pin PC15 is configured to Input disabled with pull-up */
  GPIO->P[2].DOUT |= (1 << 15);
  GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE15_MASK) | GPIO_P_MODEH_MODE15_DISABLED;
  /* Pin PD5 is configured to Input disabled with pull-up */
  GPIO->P[3].DOUT |= (1 << 5);
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_DISABLED;
  /* Pin PD6 is configured to Input disabled with pull-up */
  GPIO->P[3].DOUT |= (1 << 6);
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE6_MASK) | GPIO_P_MODEL_MODE6_DISABLED;
  /* To avoid false start, configure output US0_TX as high on PE10 */
  GPIO->P[4].DOUT |= (1 << 10);
  /* Pin PE10 is configured to Push-pull */
  GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE10_MASK) | GPIO_P_MODEH_MODE10_PUSHPULL;
  /* Pin PE11 is configured to Input disabled with pull-up */
  GPIO->P[4].DOUT |= (1 << 11);
  GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE11_MASK) | GPIO_P_MODEH_MODE11_DISABLED;
  /* Pin PE12 is configured to Push-pull */
  GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK) | GPIO_P_MODEH_MODE12_PUSHPULL;
  /* To avoid false start, configure output US0_CS as high on PE13 */
  GPIO->P[4].DOUT |= (1 << 13);
  /* Pin PE13 is configured to Push-pull */
  GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | GPIO_P_MODEH_MODE13_PUSHPULL;

  /* Set default pin positions */
  LEDS_OFF();
  DAC_LDACn_SET();
  DAC_CLRn_CLR();
  DAC_CSn_SET();
  ITIA_RELEASE();
}   

/**************************************************************************//**
 * @brief  LFTIMER_init
 * Configures and starts the LETIMER0
 *****************************************************************************/
void LFTIMER_init(void)
{    
  /* Enable clock for LETIMER0 */
  CMU->LFACLKEN0 |= CMU_LFACLKEN0_LETIMER0;
  
  /* Set initial compare values for COMP0 and COMP1 
     COMP1 keeps it's value and is used as TOP value
     for the LETIMER.
     COMP1 gets decremented through the program execution
     to generate a different PWM duty cycle */
  LETIMER_CompareSet(LETIMER0, 0, 16);
  LETIMER_CompareSet(LETIMER0, 1, 8);
  
  /* Route LETIMER to location 0 */
  LETIMER0->ROUTE = LETIMER_ROUTE_LOCATION_LOC0;
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = true,                   /* Start counting when init completed. */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOANone,        /* No output on output 0 */
  .ufoa1          = letimerUFOANone,        /* No output on output 1*/
  .repMode        = letimerRepeatFree       /* Run continuously */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
  
  /* Enable underflow, COMP1 interrupts */  
  LETIMER_IntEnable(LETIMER0, (LETIMER_IF_UF+LETIMER_IF_COMP1));  
  /* Enable LETIMER0 interrupt vector in NVIC*/
  NVIC_EnableIRQ(LETIMER0_IRQn);
}     

/**************************************************************************//**
 * @brief  HFTIMER_init
 * Configures and starts TIMER0
 *****************************************************************************/
void HFTIMER_init(void)
{       
  /* Enable clock to TIMER0 peripheral */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;

  /* Select TIMER0 parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = false, 
    .debugRun   = false, 
    .prescale   = timerPrescale32, 
    .clkSel     = timerClkSelHFPerClk, 
    .fallAction = timerInputActionNone, 
    .riseAction = timerInputActionNone, 
    .mode       = timerModeUp, 
    .dmaClrAct  = false,
    .quadModeX4 = false, 
    .oneShot    = false, 
    .sync       = false, 
  };
  
  /* Set TIMER Top value */
  TIMER_TopSet(TIMER0, 25);
  
  /* Configure TIMER */
  TIMER_Init(TIMER0, &timerInit);
}     
     
void USART0_spi_init(void)
{                               
  /* Configure USART in SPI master mode  */
  static USART_InitSync_TypeDef usartInit = {                                                                      \
    usartEnableTx,     /* Enable TX only when init completed. */                              \
    0,                 /* Use current configured reference clock for configuring baudrate. */ \
    12000000,          /* 12 Mbits/s. */                                                       \
    usartDatabits8,    /* 8 databits. */                                                      \
    true,              /* Master mode. */                                                     \
    true,              /* Send most significant bit first. */                                 \
    usartClockMode2,   /* Clock idle high, sample on falling edge. */                         \
    false,             /* Not USART PRS input mode. */                                        \
    usartPrsRxCh0,     /* PRS channel 0. */                                                   \
    false              /* No AUTOTX mode. */                                                  \
  };

    /* Enable clock for USART0 */
    CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_USART0;
                       
  /* Load SPI settings */ 
    USART_InitSync(USART0, &usartInit);

    /* Enable signals TX, CLK */
    USART0->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN;
  
}

void USART1_uart_init(void)
{                        
  /* Configure USART in UART mode at 115200 8-N-1 */
  static USART_InitAsync_TypeDef usartInit = {
    usartEnable,        /* Enable RX/TX when init completed */
    0,                  /* Use current configured reference clock for configuring baudrate */
    115200,             /* 115200 bits/s */
    usartOVS16,         /* 16x oversampling */
    usartDatabits8,     /* 8 databits */
    usartNoParity,      /* No parity */
    usartStopbits1      /* 1 stopbit */
  };                                        

  /* Enable clock for USART1 */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_USART1;
                         
  /* Load UART settings */           
  USART_InitAsync(USART1,&usartInit); 
                                   
  /* Enable signals TX, RX to location 0*/
  USART1->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN;
  
  /* Enable interrupts permanently*/
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);     
  USART1->IEN = USART_IEN_RXDATAV;               
}
               
void xDAC_init(void)
{                         
  /* Reset the DAC registers and logic levels to default */                      
  DAC_CSn_SET();                     
  DAC_LDACn_SET();                      
  DAC_CLRn_CLR();
  Delay(100);                         
  DAC_CLRn_SET();

  /* Set up the SPI peripheral */
  USART0_spi_init();

  /* Configure the DAC registers for correct operation */
}

void xDAC_write(char command, uint16_t value, bool auto_sync)
{
  /* Start the transaction */
  // TODO: change this to interrupt driven
  DAC_CSn_CLR();             
  /* Write 24 bits word */
  while (!(USART0->STATUS & USART_STATUS_TXBL)) ;
  USART0->TXDATA = command;
  while (!(USART0->STATUS & USART_STATUS_TXBL)) ;
  USART0->TXDATA = (value>>4)&0xFF;
  while (!(USART0->STATUS & USART_STATUS_TXBL)) ;
  USART0->TXDATA = (value<<4)&0xF0;  
  /*Waiting for transmission of last byte */
  while (!(USART0->STATUS & USART_STATUS_TXC)) ;  
  /* Terminate the transaction */                    
  DAC_CSn_SET();  
}

void OPA_init(void)
{                             
  /** Configuration of OPA0 as DAC0 buffer                                       */
  /* Note that when using the DAC the only relevant setting is passing output to
     OPA1 by enabling NEXTOUT                                                    */
  OPAMP_Init_TypeDef opa_config =                                                 \
  {                                                                               \
    opaNegSelUnityGain,             /* Unity gain.                             */ \
    opaPosSelDac,                   /* DAC as input.                           */ \
    opaOutModeDisable,              /* Output disabled.                        */ \
    opaResSelDefault,               /* Resistor ladder is not used.            */ \
    opaResInMuxDisable,             /* Resistor ladder disabled.               */ \
    0,                              /* No alternate outputs enabled.           */ \
    _DAC_BIASPROG_BIASPROG_DEFAULT, /* Default bias setting.             */       \
    _DAC_BIASPROG_HALFBIAS_DEFAULT, /* Default half-bias setting.        */       \
    false,                          /* No low pass filter on pos pad.          */ \
    false,                          /* No low pass filter on neg pad.          */ \
    true,                           /* Output passed to OPA1.                  */ \
    false,                          /* Neg pad disabled.                       */ \
    false,                          /* Pos pad disabled.                       */ \
    false,                          /* No shorting of inputs.                  */ \
    false,                          /* Rail-to-rail input enabled.             */ \
    true,                           /* Use factory calibrated opamp offset.    */ \
    0                               /* Opamp offset value (not available).          */ \
  };
  OPAMP_Enable(DAC0, OPA0, &opa_config); 
  /*Disable OPA0 as it is to be used by the DAC.*/
  DAC0->OPACTRL &= ~DAC_OPACTRL_OPA0EN;
       
  /** Configuration of OPA1 as DAC0 output buffer (via OPA0)                     */
  opa_config = (OPAMP_Init_TypeDef)                                               \
  {                                                                               \
    opaNegSelNegPad,                /* Neg input connected to sense pin.       */ \
    opaPosSelOpaIn,                 /* Pos input from OPA0.                    */ \
    opaOutModeAlt,                  /* Alternate output enabled.               */ \
    opaResSelDefault,               /* Resistor ladder is not used.            */ \
    opaResInMuxDisable,             /* Resistor ladder disabled.               */ \
    DAC_OPA1MUX_OUTPEN_OUT3,        /* Alternate output 3 enabled.             */ \
    _DAC_BIASPROG_BIASPROG_DEFAULT, /* Default bias setting.             */       \
    _DAC_BIASPROG_HALFBIAS_DEFAULT, /* Default half-bias setting.        */       \
    false,                          /* No low pass filter on pos pad.          */ \
    false,                          /* No low pass filter on neg pad.          */ \
    false,                          /* No nextout output enabled.              */ \
    true,                           /* Neg pad enabled, used as signal input.  */ \
    false,                          /* Pos pad disabled.                       */ \
    false,                          /* No shorting of inputs.                  */ \
    false,                          /* Rail-to-rail input enabled.             */ \
    true,                           /* Use factory calibrated opamp offset.    */ \
    0                               /* Opamp offset value (not used).          */ \
  };
  OPAMP_Enable(DAC0, OPA1, &opa_config); 
   
  /** Configuration of OPA2 as input buffer for ADC0                             */
  opa_config = (OPAMP_Init_TypeDef)                                               \
  {                                                                               \
    opaNegSelUnityGain,             /* Unity gain.                             */ \
    opaPosSelPosPad,                /* Pos input from pad.                     */ \
    opaOutModeMain,                 /* Output connected to main.               */ \
    opaResSelDefault,               /* Resistor ladder is not used.            */ \
    opaResInMuxDisable,             /* Resistor ladder disabled.               */ \
    DAC_OPA2MUX_OUTPEN_OUT1,        /* Connect to ADC0.                        */ \
    _DAC_BIASPROG_BIASPROG_DEFAULT, /* Default bias setting.             */       \
    _DAC_BIASPROG_HALFBIAS_DEFAULT, /* Default half-bias setting.        */       \
    false,                          /* No low pass filter on pos pad.          */ \
    false,                          /* No low pass filter on neg pad.          */ \
    false,                          /* No nextout output enabled.              */ \
    false,                          /* Neg pad disabled.                       */ \
    true,                           /* Pos pad enabled, used as signal input.  */ \
    false,                          /* No shorting of inputs.                  */ \
    false,                          /* Rail-to-rail input enabled.             */ \
    true,                           /* Use factory calibrated opamp offset.    */ \
    0                               /* Opamp offset value (not used).          */ \
  };
  OPAMP_Enable(DAC0, OPA2, &opa_config); 
}                 
               
void iDAC_init(void)
{                                
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_DAC0;
  /** Configuration of DAC0 for LED drive. */   
  DAC_Init_TypeDef init =  
  { 
    dacRefresh8,              /* Refresh every 8 prescaled cycles. */    \
    dacRef1V25,               /* 1.25V reference. */                       \
    dacOutputADC,             /* Required when DAC in use. */            \
    dacConvModeContinuous,    /* Continuous mode. */                     \
    0,                        /* No prescaling. */                       \
    false,                    /* Do not enable low pass filter. */       \
    false,                    /* Do not reset prescaler on ch0 start. */ \
    false,                    /* DAC output enable always on. */         \
    false,                    /* Disable sine mode. */                   \
    false                     /* Single ended mode. */                   \
  };

  /** Configuration for DAC0 channel init structure. */
  DAC_InitChannel_TypeDef initChannel =                                   \
  {
    true,               /* Enable channel when init done. */              \
    false,              /* Disable PRS triggering. */                     \
    false,              /* Channel not refreshed automatically. */        \
    dacPRSSELCh0        /* Select PRS ch0 (if PRS triggering enabled). */ \
  };

  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 1 MHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the HFPERCLK actually is. */
  init.prescale = DAC_PrescaleCalc(1000000, 0);

  /* Initialize the DAC. */
  DAC_Init(DAC0, &init);

  /* Initialise individuals DAC channels (only one here though). */
  DAC_InitChannel(DAC0, &initChannel, 0);  

  /* Enable DAC */ 
  DAC_Enable(DAC0, 0, true);

  /* Configure and enable opamps */
  OPA_init();
}

void iDAC_write(uint16_t value)
{      
  DAC0->CH0DATA = value;
}
               
void iADC_init(void)
{                                    
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_ADC0;
  /* The op-amp has already been initialised by the DAC code */   
  ADC_Init_TypeDef init =                                                    \
  { 
    adcOvsRateSel2,                /* 2x oversampling (if enabled). */       \
    adcLPFilterBypass,             /* No input filter selected. */           \
    adcWarmupNormal,               /* ADC shutdown after each conversion. */ \
    _ADC_CTRL_TIMEBASE_DEFAULT,    /* Use HW default value. */               \
    _ADC_CTRL_PRESC_DEFAULT,       /* Use HW default value. */               \
    false                          /* Do not use tailgate. */                \
  };
  ADC_InitSingle_TypeDef singleInit =                                               \
  { 
    adcPRSSELCh0,              /* PRS ch0 (if enabled). */                          \
    adcAcqTime1,               /* 1 ADC_CLK cycle acquisition time. */              \
    adcRefVDD,                 /* Buffered VDD reference. */                        \
    adcRes12Bit,               /* 12 bit resolution. */                             \
    adcSingleInpCh0,           /* CH0 input selected. */                            \
    false,                     /* Single ended input. */                            \
    false,                     /* PRS disabled. */                                  \
    false,                     /* Right adjust. */                                  \
    false                      /* Deactivate conversion after one scan sequence. */ \
  };

//    init.timebase = ADC_TimebaseCalc(0);
//    init.prescale = ADC_PrescaleCalc(7000000, 0);
    ADC_Init(ADC0, &init);
 
    ADC_InitSingle(ADC0, &singleInit);
}
