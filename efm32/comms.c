/***************************************************************************//**
 * @file
 * @brief Communications for RPOM to host
 * @author James A. C. Patterson
 * @version 0
 ******************************************************************************/
 
#include "efm32.h"
#include "comms.h"

int16_t tx_message(char_fifo_t *fifo, message_t *message) {
  /* Wait until enough space in FIFO for entire message - BLOCKING */
  // TODO: return FAIL on timeout - using systicks
  while ((MAX_FIFO_DEPTH-fifo->depth) < message->header.length) {;}
  
  /* Copy message to transmit buffer */
  if (fifo_write_n(fifo, (uint8_t *)message, message->header.length)<0) return -1;
    
  /* If the transmitter is currently idle then enables interrupts and trigger */
  if (!(USART1->IEN&USART_IEN_TXBL)) {
    USART1->TXDATA = fifo_read(fifo);
    USART1->IEN |= USART_IEN_TXBL;
    NVIC_EnableIRQ(USART1_TX_IRQn);
    // Only update pointers after successful TX
  }
  
  return 0;
}

int16_t fifo_write(char_fifo_t *fifo, uint8_t byte) {
    /* Quit if FIFO full */
    if (fifo->depth>=MAX_FIFO_DEPTH) return -1;
    /* Store the character */
    fifo->buffer[fifo->pWrite] = byte;
    /* Increment write pointer */
    if (++fifo->pWrite>=MAX_FIFO_DEPTH) fifo->pWrite = 0;
    fifo->depth++;
    return 1;
}

uint8_t fifo_read(char_fifo_t *fifo) {
  uint8_t byte;
  /* Quit if FIFO full */
  if (fifo->depth==0) return 0xFF;
  /* Read the byte */
  byte = fifo->buffer[fifo->pRead];
  /* Increment read pointer. Wrap if required. */
  if (++fifo->pRead>=MAX_FIFO_DEPTH) fifo->pRead=0;
  fifo->depth--;
  return byte;
}

int16_t fifo_write_n(char_fifo_t *fifo, uint8_t *buffer, uint16_t length) {
  uint16_t i = 0;
  uint8_t *pwrite;
  /* Protect against invalid messages */
  if (length>MAX_FIFO_DEPTH) return -1;
  /* Copy the buffer to the FIFO */
  pwrite = (fifo->buffer + fifo->pWrite);
  for (i=0;i<length;i++) {
    *pwrite++ = *buffer++;
    /* Wrap the pointer at the end of the buffer array */
    if (pwrite>=(fifo->buffer+MAX_FIFO_DEPTH))
      pwrite = fifo->buffer;
  }
  /* Increment pointers. Wrap if required. */
  fifo->pWrite += length;
  if (fifo->pWrite>=MAX_FIFO_DEPTH)
    fifo->pWrite-=MAX_FIFO_DEPTH;
  fifo->depth += length;
  return length;
}

int16_t fifo_read_n(char_fifo_t *fifo, uint8_t *buffer, uint16_t length) {
  uint16_t i = 0;
  uint8_t *pread;
  /* Protect against silly requests */
  if (length>fifo->depth) return -1;
  /* Copy the buffer to the FIFO */
  pread = (fifo->buffer + fifo->pRead);
  for (i=0;i<length;i++) {
    *buffer++ = *pread++;
    /* Wrap the pointer at the end of the buffer array */
    if (pread>=(fifo->buffer+MAX_FIFO_DEPTH))
      pread = fifo->buffer;
  }
  /* Increment pointers. Wrap if required. */
  fifo->pRead += length;
  if (fifo->pRead>=MAX_FIFO_DEPTH)
    fifo->pRead-=MAX_FIFO_DEPTH;
  fifo->depth -= length;
  return length;
}
struct message_filter_context_s {
	int state;
	uint8_t buffer[MAX_MESSAGE_LEN];
	uint16_t bdepth;  
};
static struct message_filter_context_s mf;
/* Reset state machine */
void reset_parser(void) {
		mf.state = 0;
		mf.bdepth = 0;
}
uint16_t message_parser(char_fifo_t* fifo, message_t** outmessage) {
	uint16_t i;
  uint16_t br,desired;
  int16_t required;
  uint8_t *pBuffer = mf.buffer;
  /* Serialise the packet as an octet stream */
	message_t* pPacket = (message_t*)mf.buffer;
	// TODO: Test if there has been an overrun
	switch (mf.state) {
	case 0:		// Test for valid 4 byte header
		/* Read required number of bytes from the fifo */
		if (mf.bdepth < sizeof(pPacket->header)) {
			desired = sizeof(pPacket->header)-mf.bdepth;
			if (desired > fifo->depth) return 0;
      br = fifo_read_n(fifo,((uint8_t*)mf.buffer+mf.bdepth), desired);
			mf.bdepth += br;
			/* Return if there are still not enough bytes to parse a header */
			if (mf.bdepth < sizeof(pPacket->header)) return 0;
		}
		/* Test for invalid ID, TYPE and LENGTH */
		if ((pPacket->header.id >= 16) ||
				(pPacket->header.type > 32) ||
				(pPacket->header.length < 6) ||
				(pPacket->header.length > 64)	) {
			/* Shift buffer by 1 byte and wait for another */
			for (i=0;i<mf.bdepth-1;i++)
        *pBuffer++ = *(pBuffer+1);
			/* Record number of bytes already waiting */
			mf.bdepth--;
			return 0;
		}
		/* No need to test CHECKSUM until payload has been received
		   After a valid header look for the body */
		mf.state = 1;
		/* Test if there are already enough bytes to check the payload */
		required = pPacket->header.length - mf.bdepth;
		if (required>fifo->depth) return 0;
		/* Otherwise head straight into the next state */
	case 1:		// PAYLOAD received
		/* Make sure there are enough bytes */
		if (mf.bdepth < pPacket->header.length) {
			desired = pPacket->header.length-mf.bdepth;
			if (desired > fifo->depth) return 0;
      br = fifo_read_n(fifo,((uint8_t*)mf.buffer+mf.bdepth), desired);
			mf.bdepth += br;
			if (mf.bdepth < pPacket->header.length)
				return 0;
		}
		/* Always return to looking for a header after this state */
		mf.state = 0;
		/* TODO: Calculate & test checksum */
		if (pPacket->header.check != 0) {
			/* - If checksum is invalid then need to search the packet for another valid header
			   - Do this by shifting the buffer up and recursively calling this function */
			for (i=0;i<mf.bdepth-1;i++)
        *pBuffer++ = *(pBuffer+1);
			mf.bdepth -= 1;
			return 0;
		}
		/* "Return" a pointer to the valid packet */
		*outmessage = (message_t *)mf.buffer;
		/* After a valid body look for the next header */
		mf.bdepth -= pPacket->header.length;
		/* Realign buffer to point to remaining byte */
		for (i=0;i<mf.bdepth-1;i++)
      *pBuffer++ = *(pBuffer+pPacket->header.length);
    /* By returning a positive number we confirm that a valid packet is in buffer */
		return pPacket->header.length;
	default:	// INVALID state protection
    reset_parser();
		return 0;
	}
}
