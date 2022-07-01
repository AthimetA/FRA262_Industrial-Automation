/*
 * uartRingBufDMA.h
 *
 *  Created on: Aug 12, 2021
 *      Author: controllerstech.com
 */

#ifndef INC_UARTRINGBUFDMA_H_
#define INC_UARTRINGBUFDMA_H_


/* Initialize the Ring buffer
 * It will also initialize the UART RECEIVE DMA
 * */
void Ringbuf_Init (void);

/* Reset the ring buffer
 * Resets the Head and Tail, also the buffers
 * */
void Ringbuf_Reset (void);

void checkSum (uint8_t *buffertoCheckSum, uint16_t Size);
uint8_t checkAck(uint8_t *buffertoCheckAck, uint16_t Size);

#endif /* INC_UARTRINGBUFDMA_H_ */
