/*
 * wifi_at.h
 *
 *  Created on: Aug 24, 2022
 *      Author: a
 */

#ifndef INC_WIFI_AT_H_
#define INC_WIFI_AT_H_

#include <stdlib.h>
#include <stdint.h>
#include "cmsis_os.h"

#define WIFI_AT_DEFAULT_TMT	(pdMS_TO_TICKS(1000))

typedef enum {
	WIFI_OK = 0,		// OK
	WIFI_ERR_IO,		// IO error
	WIFI_ERR_TMT,		// timeout
	WIFI_ERR_NO_IP,		// can't get IP
	WIFI_ERR_NO_AP,		// can't connect to AP
	WIFI_ERR_GENEGAL	// some other error
} wifi_status_t;

typedef enum {
	WIFI_MODE_NONE = 0,
	WIFI_MODE_STA = 1,
	WIFI_MODE_AP = 2,
	WIFI_MODE_HYBRID = 3,
} wifi_mode_t;

int wifi_init(void);
int wifi_set_mode(wifi_mode_t mode);
int wifi_connect_to_ap(char *ap_name, char *pass, uint32_t tmt);
int wifi_get_own_ip(uint32_t *ip, uint32_t *mac);
int wifi_multiple_conns(int multiple);
int wifi_server(int create, int port);
int wifi_send(int link, char *text, uint32_t len);
int wifi_printf(int link, char *text, ...);
int wifi_close(int link);

#endif /* INC_WIFI_AT_H_ */
