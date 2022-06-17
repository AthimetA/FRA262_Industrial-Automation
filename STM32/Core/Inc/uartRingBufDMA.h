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

/* waits for a particular string in the Rx Buffer
 * By Default it is set to wait for "OK", you can change the string in the HAL_UARTEx_RxEventCallback function
 * This function will wait in the blocking mode, so to avoid the halt, we will also include the timeout
 * The timeout is in milliseconds
 * returns 1 on success
 * returns 0 on failure
 * */
uint8_t isConfirmed (int32_t Timeout);

uint8_t checkSum (uint8_t *buffertoCheckSum , int bufferSize);


#endif /* INC_UARTRINGBUFDMA_H_ */
