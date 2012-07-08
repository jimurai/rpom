/***************************************************************************//**
 * @file
 * @brief Communications for RPOM to host
 * @author James A. C. Patterson
 * @version r0
 ******************************************************************************/
#ifndef __COMMS_H
#define __COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define UART_BUFFER_DEPTH 32
typedef struct { 
  uint8_t pRead;  
  uint8_t pWrite;
  uint8_t depth;
  uint8_t buffer[UART_BUFFER_DEPTH];
} char_fifo_t;
  
#define MAX_MESSAGE_LEN 32
typedef struct {
   uint8_t id;  
   uint8_t type;
   uint8_t length;          
   uint8_t check;
} header_t;
typedef struct {
   header_t header;
   uint8_t payload[MAX_MESSAGE_LEN-sizeof(header_t)];
} message_t; 

int tx_message(message_t *message, char_fifo_t *fifo);

#ifdef __cplusplus
}
#endif

#endif /* __COMMS_H */
