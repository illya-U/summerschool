/*
 * streams.h
 *
 *  Created on: Aug 25, 2022
 *      Author: a
 */

#ifndef INC_STREAMS_H_
#define INC_STREAMS_H_

#include "main.h"
#include <stm32f4xx_hal.h>
#include <unistd.h>

#define UART_RX_BUF_SIZE	2048

enum {
	FILE_STDIN = STDIN_FILENO,
	FILE_STDOUT = STDOUT_FILENO,
	FILE_STDERR = STDERR_FILENO,
	FILE_WIFI,
};

enum {
	BUF_STDIO,
	BUF_WIFI,
	BUF_MAX
};

typedef struct {
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma;
	volatile uint8_t *buffer;
	volatile uint8_t *head;
	volatile uint8_t *tail;
	void *mutex_tx;
	void *mutex_rx;
	void *sem;
	uint32_t size;
} buff_attr_t;

extern buff_attr_t buffer[BUF_MAX];

void streams_init(void);

#endif /* INC_STREAMS_H_ */
