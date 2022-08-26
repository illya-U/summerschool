/*
 * wifi_at.c
 *
 *  Created on: Aug 24, 2022
 *      Author: a
 */

#include "main.h"
#include "wifi_at.h"
#include "streams.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define UART_BUF_SIZE	300
static char rx_buffer[UART_BUF_SIZE];
static char tx_buffer[UART_BUF_SIZE];
static FILE *fp;
static int mux = 0;

extern volatile uint32_t pressure;

enum {
	AT_RESP_OK,
	AT_RESP_ERROR,
	AT_RESP_FAIL,
	AT_RESP_WIFI_CONNECTED,
	AT_RESP_WIFI_GOT_IP,
	AT_RESP_WIFI_DISCONNECT,
	AT_RESP_UNLINK,
	AT_RESP_SEND_OK,

	AT_MAX_RESPONSES
};

static const char *at_response[AT_MAX_RESPONSES] = {
	[AT_RESP_OK] = "OK",
	[AT_RESP_ERROR] = "ERROR",
	[AT_RESP_FAIL] = "FAIL",
	[AT_RESP_WIFI_CONNECTED] = "WIFI CONNECTED",
	[AT_RESP_WIFI_GOT_IP] = "WIFI GOT IP",
	[AT_RESP_WIFI_DISCONNECT] = "WIFI DISCONNECT",
	[AT_RESP_UNLINK] = "UNLINK",
	[AT_RESP_SEND_OK] = "SEND OK"
};

static int cmp_resp(char *text, int num) {
	return strncmp(text, at_response[num], strlen(at_response[num]));
}

#define LISTENER_BUF_SIZE	500
static uint8_t listener_buf[LISTENER_BUF_SIZE];
static char html[] =
"<!DOCTYPE html>\
<html>\
<body>\
    <h1>WiFi embedded server</h1>\
    <p>Pressure: %ld.%02ld hPa</p>\
</body>\
</html>";

void StartTaskWiFiListen(void *argument) {
	while (1) {
		osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);
		volatile uint8_t *head;
		head = &buffer[BUF_WIFI].buffer[UART_RX_BUF_SIZE - buffer[BUF_WIFI].hdma->Instance->NDTR];
		int link = 0;
		if (buffer[BUF_WIFI].tail != head) {
			do {
				head = &buffer[BUF_WIFI].buffer[UART_RX_BUF_SIZE - buffer[BUF_WIFI].hdma->Instance->NDTR];
				memset(listener_buf, 0, LISTENER_BUF_SIZE);
				uint32_t bytes = (uint32_t)buffer[BUF_WIFI].tail + buffer[BUF_WIFI].size - (uint32_t)head;
				fgets((char*)listener_buf, bytes, fp);
				if (strstr((char*)listener_buf, ",CONNECT")) {
					sscanf((char*)listener_buf, "%d,CONNECT", &link);
				}
				if (strncmp((char*)listener_buf, "+IPD,", 5) == 0) {
					osDelay(pdMS_TO_TICKS(100));
					// HOORAY!
					osMutexRelease(buffer[BUF_WIFI].mutex_rx);
					if (wifi_printf(link, html, pressure/100, pressure%100) == WIFI_OK) {
						wifi_close(link);
					}
					osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);
				}

			} while (buffer[BUF_WIFI].tail != head);
		}
		osMutexRelease(buffer[BUF_WIFI].mutex_rx);
		osDelay(pdMS_TO_TICKS(10));
	}
}

/******************************************************************/
int wifi_init(void) {
	int ret = WIFI_OK;

	fp = fopen("wifi", "wb+");
	if (!fp)
		return WIFI_ERR_IO;

	if (setvbuf(fp, NULL, _IONBF, 0) != 0) {
		while (1) {}
	}

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);

	fprintf(fp, "ATE0\r\n");

	TickType_t tmt_time = xTaskGetTickCount() + pdMS_TO_TICKS(20000);
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0)
			break;
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}

	fprintf(fp, "AT+RST\r\n");
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0)
			break;
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}

	// Discard all junk
	HAL_UART_Abort(buffer[BUF_WIFI].huart);
	buffer[BUF_WIFI].head = buffer[BUF_WIFI].tail = buffer[BUF_WIFI].buffer;

	osDelay(pdMS_TO_TICKS(8000));
	HAL_StatusTypeDef hal_st = HAL_UART_Receive_DMA(buffer[BUF_WIFI].huart,
													(uint8_t*)buffer[BUF_WIFI].buffer,
													buffer[BUF_WIFI].size);
	if (hal_st != HAL_OK) {
		ret = WIFI_ERR_IO;
		goto quit;
	}

	fprintf(fp, "ATE0\r\n");
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0)
			break;
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);

	wifi_set_mode(WIFI_MODE_STA);

quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}

int wifi_set_mode(wifi_mode_t mode) {
	wifi_status_t ret = WIFI_OK;

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);

	fprintf(fp, "AT+CWMODE=%d\r\n", mode);

	TickType_t tmt_time = xTaskGetTickCount() + WIFI_AT_DEFAULT_TMT;
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0)
			break;
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}

quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}

int wifi_connect_to_ap(char *ap_name, char *pass, uint32_t tmt) {
	uint32_t flags = 0;
	wifi_status_t ret = WIFI_OK;

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);

	fprintf(fp, "AT+CWJAP=\"%s\",\"%s\"\r\n", ap_name, pass);

	TickType_t tmt_time = xTaskGetTickCount() + tmt;
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0) {
			if (flags & 0x3) {
				goto quit;
			}
			else {
				if (!(flags & 0x01)) {
					ret = WIFI_ERR_NO_AP;
					goto quit;
				}
				if (!(flags & 0x02)) {
					ret = WIFI_ERR_NO_IP;
					goto quit;
				}
				ret = WIFI_ERR_GENEGAL;
				goto quit;
			}
		}
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
		if (cmp_resp(rx_buffer, AT_RESP_FAIL) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
		if (cmp_resp(rx_buffer, AT_RESP_WIFI_CONNECTED) == 0)
			flags |= 0x01;
		if (cmp_resp(rx_buffer, AT_RESP_WIFI_GOT_IP) == 0)
			flags |= 0x02;
	}

quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}

int wifi_get_own_ip(uint32_t *ip, uint32_t *mac) {
	uint32_t flags = 0;

	wifi_status_t ret = WIFI_OK;

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);

	fprintf(fp, "AT+CIFSR\r\n");

	TickType_t tmt_time = xTaskGetTickCount() + WIFI_AT_DEFAULT_TMT;
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0) {
			if (flags & 0x03) {
				ret = WIFI_OK;
				goto quit;
			} else {
				ret = WIFI_ERR_NO_IP;
				goto quit;
			}
		}
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}

		if (strncmp(rx_buffer, "+CIFSR:STAIP", sizeof("+CIFSR:STAIP")-1) == 0) {
			flags |= 0x01;
			sscanf(rx_buffer, "+CIFSR:STAIP,\"%lu.%lu.%lu.%lu\"",
					&ip[0], &ip[1], &ip[2], &ip[3]);
		}
		if (strncmp(rx_buffer, "+CIFSR:STAMAC", sizeof("+CIFSR:STAMAC")-1) == 0) {
			flags |= 0x02;
			sscanf(rx_buffer, "+CIFSR:STAMAC,\"%02lX:%02lX:%02lX:%02lX:%02lX:%02lX\"",
					&mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
		}
	}
quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}

int wifi_multiple_conns(int multiple) {
	wifi_status_t ret = WIFI_OK;

	mux = multiple;

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);
	fprintf(fp, "AT+CIPMUX=%d\r\n", multiple);

	TickType_t tmt_time = xTaskGetTickCount() + WIFI_AT_DEFAULT_TMT;
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0)
			break;
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}

quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}

int wifi_server(int create, int port) {
	wifi_status_t ret;

	ret = wifi_multiple_conns(1);
	if (ret) {
		return ret;
	}

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);

	if (create)
		fprintf(fp, "AT+CIPSERVER=%d,%d\r\n", create, port);
	else
		fprintf(fp, "AT+CIPSERVER=0\r\n");

	TickType_t tmt_time = xTaskGetTickCount() + WIFI_AT_DEFAULT_TMT;
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0)
			break;
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}

quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}

int wifi_send(int link, char *text, uint32_t len) {
	uint32_t flags = 0;
	wifi_status_t ret = WIFI_OK;

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);

	if (mux)
		fprintf(fp, "AT+CIPSEND=%d,%ld\r\n", link, len);
	else
		fprintf(fp, "AT+CIPSEND=%ld\r\n", len);

	TickType_t tmt_time = xTaskGetTickCount() + WIFI_AT_DEFAULT_TMT;
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0) {
			fwrite(text, 1, len, fp);
			flags |= 1;
			continue;
		}
		if (flags) {
			if (cmp_resp(rx_buffer, AT_RESP_SEND_OK) == 0) {
				break;
			}
		}
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}

quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}

int wifi_printf(int link, char *fmt, ...) {
	int len = 0;
	va_list args;
	memset(tx_buffer, 0, sizeof(rx_buffer));
	va_start(args, fmt);
	vsprintf(tx_buffer, fmt, args);
	va_end(args);
	len = strnlen(tx_buffer, sizeof(tx_buffer));
	return wifi_send(link, tx_buffer, len);
}

int wifi_close(int link) {
	wifi_status_t ret = WIFI_OK;

	osMutexAcquire(buffer[BUF_WIFI].mutex_rx, osWaitForever);

	fprintf(fp, "AT+CIPCLOSE=%d\r\n", link);

	TickType_t tmt_time = xTaskGetTickCount() + WIFI_AT_DEFAULT_TMT;
	while (1) {
		if (xTaskGetTickCount() >= tmt_time) {
			ret = WIFI_ERR_TMT;
			goto quit;
		}

		memset(rx_buffer, 0, sizeof(rx_buffer));
		fgets(rx_buffer, UART_BUF_SIZE, fp);
		if (cmp_resp(rx_buffer, AT_RESP_OK) == 0)
			break;
		if (cmp_resp(rx_buffer, AT_RESP_ERROR) == 0) {
			ret = WIFI_ERR_GENEGAL;
			goto quit;
		}
	}

quit:
	osMutexRelease(buffer[BUF_WIFI].mutex_rx);
	return ret;
}
