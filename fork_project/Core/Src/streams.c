/*
 * streams.c
 *
 *  Created on: Aug 25, 2022
 *      Author: a
 */

#include "streams.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

volatile uint8_t rx_dma_buffer[BUF_MAX][UART_RX_BUF_SIZE];

buff_attr_t buffer[BUF_MAX] = {
	[BUF_STDIO] = {
		.huart = &huart1,
		.hdma = &hdma_usart1_rx,
		.buffer = rx_dma_buffer[BUF_STDIO],
		.head = rx_dma_buffer[BUF_STDIO],
		.tail = rx_dma_buffer[BUF_STDIO],
		.size = UART_RX_BUF_SIZE,
	},
	[BUF_WIFI] = {
		.huart = &huart6,
		.hdma = &hdma_usart6_rx,
		.buffer = rx_dma_buffer[BUF_WIFI],
		.head = rx_dma_buffer[BUF_WIFI],
		.tail = rx_dma_buffer[BUF_WIFI],
		.size = UART_RX_BUF_SIZE,
	},
};

osMutexId_t muxUART_STDIO_TXHandle;
const osMutexAttr_t muxUART_STDIO_TX_attributes = {
  .name = "muxUART_STDIO_TX"
};

osMutexId_t muxUART_STDIO_RXHandle;
const osMutexAttr_t muxUART_STDIO_RX_attributes = {
  .name = "muxUART_STDIO_RX"
};

osMutexId_t muxUART_WIFI_TXHandle;
const osMutexAttr_t muxUART_WIFI_TX_attributes = {
  .name = "muxUART_WIFI_TX"
};

osMutexId_t muxUART_WIFI_RXHandle;
const osMutexAttr_t muxUART_WIFI_RX_attributes = {
  .name = "muxUART_WIFI_RX"
};

osSemaphoreId_t semUART_WIFIHandle;
const osSemaphoreAttr_t semUART_WIFI_attributes = {
  .name = "semUART_WIFI"
};

osSemaphoreId_t semUART_STDIOHandle;
const osSemaphoreAttr_t semUART_STDIO_attributes = {
  .name = "semUART_STDIO"
};

int __io_getchar(int file) {
	int buf;
	if (file == FILE_STDIN)
		buf = BUF_STDIO;
	else if (file == FILE_WIFI)
		buf = BUF_WIFI;
	else
		return EOF;

	buffer[buf].head = &buffer[buf].buffer[UART_RX_BUF_SIZE - buffer[buf].hdma->Instance->NDTR];

	while (buffer[buf].tail == buffer[buf].head) {
		buffer[buf].head = &buffer[buf].buffer[UART_RX_BUF_SIZE - buffer[buf].hdma->Instance->NDTR];
		osDelay(1);
	}

	uint8_t b = *buffer[buf].tail;

	if (++buffer[buf].tail == (buffer[buf].buffer + UART_RX_BUF_SIZE))
		buffer[buf].tail = buffer[buf].buffer;

	return (int)b;
}

int __io_putchar(int file, int ch) {
	int buf;
	if (file == FILE_STDOUT || file == FILE_STDERR)
		buf = BUF_STDIO;
	else if (file == FILE_WIFI)
		buf = BUF_WIFI;
	else
		return 0;

	osMutexAcquire(buffer[buf].mutex_tx, osWaitForever);
	HAL_UART_Transmit_IT(buffer[buf].huart, (uint8_t*)&ch, 1);
	osSemaphoreAcquire(buffer[buf].sem, osWaitForever);
	osMutexRelease(buffer[buf].mutex_tx);
	return 0;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		osSemaphoreRelease(semUART_STDIOHandle);
	}

	if (huart == &huart6) {
		osSemaphoreRelease(semUART_WIFIHandle);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	int b;
	if (huart == &huart1)
		b = BUF_STDIO;
	else if (huart == &huart6)
		b = BUF_WIFI;
	else
		return;

	// Stop UART reception
	HAL_UART_Abort(huart);

	/*
	*            @arg UART_FLAG_ORE:  Overrun Error flag
	*            @arg UART_FLAG_NE:   Noise Error flag
	*            @arg UART_FLAG_FE:   Framing Error flag
	*            @arg UART_FLAG_PE:   Parity Error flag
	*/
	// Clear error flags
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE);

	// Reset ring buffer
	buffer[b].head = buffer[b].tail = buffer[b].buffer;
	// Restart UART reception
	HAL_UART_Receive_IT(huart, (uint8_t*)buffer[b].buffer, buffer[b].size);
}

/*********************************************************************/
/* INIT                                                              */
/*********************************************************************/
void streams_init(void) {
	muxUART_STDIO_TXHandle = osMutexNew(&muxUART_STDIO_TX_attributes);
	muxUART_STDIO_RXHandle = osMutexNew(&muxUART_STDIO_RX_attributes);
	muxUART_WIFI_TXHandle = osMutexNew(&muxUART_WIFI_TX_attributes);
	muxUART_WIFI_RXHandle = osMutexNew(&muxUART_WIFI_RX_attributes);

	semUART_WIFIHandle = osSemaphoreNew(1, 0, &semUART_WIFI_attributes);
	semUART_STDIOHandle = osSemaphoreNew(1, 0, &semUART_STDIO_attributes);

	buffer[BUF_STDIO].sem = semUART_STDIOHandle;
	buffer[BUF_STDIO].mutex_rx = muxUART_STDIO_RXHandle;
	buffer[BUF_STDIO].mutex_tx = muxUART_STDIO_TXHandle;
	buffer[BUF_WIFI].sem = semUART_WIFIHandle;
	buffer[BUF_WIFI].mutex_rx = muxUART_WIFI_RXHandle;
	buffer[BUF_WIFI].mutex_tx = muxUART_WIFI_TXHandle;

	for (uint32_t b = 0; b < BUF_MAX; b++) {
		if (HAL_UART_Receive_DMA(buffer[b].huart,
								 (uint8_t*)buffer[b].buffer,
								 buffer[b].size) != HAL_OK) {
			printf("Error start UART RX %ld\n", b);
			while (1) {}
		}
	}
}
