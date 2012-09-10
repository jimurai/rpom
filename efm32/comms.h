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

#define MAX_FIFO_DEPTH 64
typedef struct { 
  uint8_t pRead;  
  uint8_t pWrite;
  uint8_t depth;
  uint8_t buffer[MAX_FIFO_DEPTH];
} char_fifo_t;
  
#define MAX_MESSAGE_LEN 64
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

int16_t tx_message(char_fifo_t *fifo, message_t *message);
int16_t fifo_write(char_fifo_t *fifo, uint8_t byte);
uint8_t fifo_read(char_fifo_t *fifo);
int16_t fifo_write_n(char_fifo_t *fifo, uint8_t *buffer, uint16_t length);
int16_t fifo_read_n(char_fifo_t *fifo, uint8_t *buffer, uint16_t length);
uint16_t message_parser(char_fifo_t *fifo, message_t **outmessage);

#ifdef __cplusplus
}
#endif

#endif /* __COMMS_H */
