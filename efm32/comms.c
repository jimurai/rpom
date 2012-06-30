/***************************************************************************//**
 * @file
 * @brief Communications for RPOM to host
 * @author James A. C. Patterson
 * @version 0
 ******************************************************************************/
 
#include "efm32.h"
#include "comms.h"

int tx_message(message_t *message, char_fifo_t *fifo) {              
  /* Protect against invalid messages */
  if (message->header.length>UART_BUFFER_DEPTH) return -1;
  
  /* Wait until enough space in FIFO for entire message - BLOCKING */
  // TODO: return FAIL on timeout - using systicks
  while ((UART_BUFFER_DEPTH-fifo->depth) < message->header.length) {;}
  
  /* Copy message to transmit buffer */
  unsigned int i = 0;
  uint8_t *pwrite = (fifo->buffer + fifo->pWrite);
  uint8_t *pread = &message->header.id;
  for (i=0;i<message->header.length;i++) {
    *pwrite++ = *pread++;
  }
  /* Increment write pointer. Wrap if required. */
  fifo->pWrite += message->header.length;
  if (fifo->pWrite>=UART_BUFFER_DEPTH) fifo->pWrite-=UART_BUFFER_DEPTH;
  
  /* If the transmitter is currently idle then enables interrupts and trigger */
  if (fifo->depth == 0) {
    fifo->depth = message->header.length;
    USART1->TXDATA = fifo->buffer[fifo->pRead];
    USART1->IEN |= USART_IEN_TXBL;
    NVIC_EnableIRQ(USART1_TX_IRQn);
    // Only update pointers after successful TX
  }
  else {
    fifo->depth += message->header.length;
  }
  return 0;
  
}
