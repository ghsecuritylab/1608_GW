/*
 * eth_app.h
 *
 *  Created on: 2018Äê5ÔÂ23ÈÕ
 *      Author: Snail
 */

#ifndef INC_ETH_APP_H_
#define INC_ETH_APP_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lwip/netif.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* DHCP process states */
#define DHCP_OFF                   (uint8_t) 0
#define DHCP_START                 (uint8_t) 1
#define DHCP_WAIT_ADDRESS          (uint8_t) 2
#define DHCP_ADDRESS_ASSIGNED      (uint8_t) 3
#define DHCP_TIMEOUT               (uint8_t) 4
#define DHCP_LINK_DOWN             (uint8_t) 5


 /* TCP run mode*/
#define CLNT_MODE  0
#define SRV_MODE   1
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void User_notification(struct netif *netif);

void DHCP_thread(void const * argument);

void msg_send(uint8_t *tx_buf, uint16_t tx_size, uint8_t Mode);

void vtcp_client(void * argument);

void task_server_socket(void *argument);

void server_recv_data(int conn);

#ifdef __cplusplus
}
#endif

#endif /* INC_ETH_APP_H_ */
