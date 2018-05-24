/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "ethernetif.h"
#include "eth_app.h"

/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void _Error_Handler(char * file, int line);

/* Semaphore to signal Ethernet Link state update */
osSemaphoreId Netif_LinkSemaphore = NULL;
/* Ethernet link thread Argument */
struct link_str link_arg;

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;

uint8_t lan_flag = 0;
extern __IO uint8_t DHCP_state;
/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  tcpip_init( NULL, NULL );

  ipaddr.addr = 0;
  netmask.addr = 0;
  gw.addr = 0;

  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
	 netif_set_up(&gnetif);
	 lan_flag = 1;
  }
  else
  {
    netif_set_down(&gnetif);
  }

  netif_set_link_callback(&gnetif, ethernetif_update_config);

  /*创建一个用于通知帧接收的二进制信号量 */
  osSemaphoreDef(Netif_SEM);
  Netif_LinkSemaphore = osSemaphoreCreate(osSemaphore(Netif_SEM) , 1 );

  link_arg.netif = &gnetif;
  link_arg.semaphore = Netif_LinkSemaphore;

  /* 创建以太网连接处理程序读取PHY状态防止掉线 */
  osThreadDef(LinkThr, ethernetif_set_link, osPriorityNormal, 0, 128);
  osThreadCreate (osThread(LinkThr), &link_arg);

  User_notification(&gnetif);

  osThreadDef(DHCP, DHCP_thread, osPriorityBelowNormal, 0, 128);
  osThreadCreate (osThread(DHCP), &gnetif);
}

void ethernetif_notify_conn_changed(struct netif *netif)
{
  //uart_send_buf( uart1_t, "重新连接\r\n", 10);
  if( lan_flag == 0 )
  {
    netif_set_up(netif);
  }
  DHCP_state = DHCP_START;
}
