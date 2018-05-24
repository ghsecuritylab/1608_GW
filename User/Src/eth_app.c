/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "main.h"
#include "lwip/dhcp.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/api.h"
#include "eth_app.h"
#include "ethernetif.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "device.h"

extern device_info_t device_info;
extern ip_addr_t              dns_servers[DNS_MAX_SERVERS];

#define MAX_DHCP_TRIES  3
#define TCP_CLIENT_RX_BUFSIZE 1024
__IO uint8_t DHCP_state = DHCP_OFF;

struct netconn *connect;

void msg_send(uint8_t *tx_buf, uint16_t tx_size, uint8_t mode)
{
  if(mode == CLNT_MODE)
  {
    netconn_write( connect, tx_buf, tx_size ,NETCONN_COPY);
  }
  else if(mode == SRV_MODE)
  {
    netconn_write( connect, tx_buf, tx_size ,NETCONN_COPY);
  }
}


void User_notification(struct netif *netif)
{
  if (netif_is_up(netif))
  {
    DHCP_state = DHCP_START;

  }
  else
  {
    DHCP_state = DHCP_LINK_DOWN;
    //uart_send_buf( uart1_t, (uint8_t *)"网口未连接\r\n", 12 );
  }
}

void DHCP_thread(void const * argument)
{
  struct netif *netif = (struct netif *) argument;
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp;

  for (;;)
  {
    switch (DHCP_state)
    {
    case DHCP_START:
      {
          if(device_info.save_params.dhcp_en == 1)
          {
            DHCP_state = DHCP_ADDRESS_ASSIGNED;
            dhcp_stop(netif);

           /*字符串转换IPV4成功加入地址*/
            ip4addr_aton(device_info.save_params.local_ip, &ipaddr);
            ip4addr_aton(device_info.save_params.local_gateway, &gw) ;
            ip4addr_aton(device_info.save_params.local_mask, &netmask);

            netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
            if(device_info.run_params.create_tcp == 0)
            {
              if(device_info.save_params.device_mod == CLNT_MODE)
              {
                xTaskCreate( vtcp_client, "tcp_client", 2048,NULL,1,NULL );
              }
              else
              {
                xTaskCreate( task_server_socket, "tcp_server", 512, NULL, 2, NULL);
              }
            }
          }
         else
         {
           ip_addr_set_zero_ip4(&netif->ip_addr);
           ip_addr_set_zero_ip4(&netif->netmask);
           ip_addr_set_zero_ip4(&netif->gw);
           dhcp_start(netif);
           DHCP_state = DHCP_WAIT_ADDRESS;
         }
      }
      break;

    case DHCP_WAIT_ADDRESS:
      {
        if (dhcp_supplied_address(netif))
        {
          DHCP_state = DHCP_ADDRESS_ASSIGNED;

          sprintf( device_info.save_params.local_ip, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->ip_addr));
          sprintf( device_info.save_params.local_mask, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->netmask));
          sprintf( device_info.save_params.local_gateway, "%s", ip4addr_ntoa((const ip4_addr_t *)&netif->gw));
          sprintf( device_info.save_params.local_dns_addr, "%s", ip4addr_ntoa((const ip4_addr_t *)&dns_servers[0]));

          if(device_info.save_params.device_mod == CLNT_MODE)
          {
            xTaskCreate( vtcp_client, "tcp_client", 2048,NULL,1,NULL );
          }
          else
          {
            xTaskCreate( task_server_socket, "tcp_server", 512, NULL, 2, NULL);
          }
        }
        else
        {
          dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);

          /* DHCP timeout */
          if (dhcp->tries > MAX_DHCP_TRIES)
          {
            DHCP_state = DHCP_TIMEOUT;
            /* Stop DHCP */
            dhcp_stop(netif);
            //uart_send_buf( uart1_t,"TIME OUT\r\n",10 );
            ip4addr_aton(device_info.save_params.local_ip, &ipaddr);
            ip4addr_aton(device_info.save_params.local_gateway, &gw) ;
            ip4addr_aton(device_info.save_params.local_mask, &netmask);
            netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));

            ip_addr_t dnsserver;
            /*不启动DHCP的情况下手动设置DNS地址*/
            ip4addr_aton(device_info.save_params.local_dns_addr, &dnsserver);
            dns_setserver(0, &dnsserver);
            if(device_info.save_params.device_mod == CLNT_MODE)
            {
              xTaskCreate( vtcp_client, "tcp_client", 2048,NULL,1,NULL );
            }
            else
            {
              xTaskCreate( task_server_socket, "tcp_server", 512, NULL, 2, NULL);
            }
          }
        }
      }
      break;
  case DHCP_LINK_DOWN:
    {
      dhcp_stop(netif);
      DHCP_state = DHCP_OFF;
    }
    break;
    default: break;
    }

    /* wait 250 ms */
    osDelay(250);
  }
}


void vtcp_client(void * argument)
{
  uint32_t ulLnegth = 0;
  err_t err,recv_err;
  static uint16_t uslocal_port;
  static ip_addr_t server_ipaddr,loca_ipaddr;
  struct pbuf *q;

  struct netbuf *recvbuf;

  uint8_t *TCP_Client_Recv;

  device_info.run_params.create_tcp = 1;

  TCP_Client_Recv = pvPortMalloc( 1024 );

  while(1)
  {
    netconn_gethostbyname( (const char *)device_info.save_params.server_addr, &server_ipaddr);

    if(server_ipaddr.addr == 0X00)
    {
      vTaskDelay(500);
      //uart_send_buf( uart1_t, "DNS失败\r\n", 11 );
      continue;
    }
    connect = netconn_new( NETCONN_TCP );
    connect->recv_timeout = 20;
    err = netconn_connect(connect, &server_ipaddr ,atoi(device_info.save_params.server_port) );

    if(err != ERR_OK)
    {
      netconn_close(connect);
      netconn_delete(connect);
      if(err == ERR_RTE)
      {
        osDelay(10);
      }
      osDelay(10);
    }
    else if(err == ERR_OK)
    {
      //uart_send_buf( uart1_t, "连接服务器成功\r\n", 16 );
      netconn_getaddr( connect, &loca_ipaddr, &uslocal_port, 1 );
      netconn_write( connect, device_info.save_params.serial_num, strlen(device_info.save_params.serial_num), NETCONN_COPY );
      osDelay(100);
      while(1)
      {
       if((recv_err = netconn_recv(connect, &recvbuf)) == ERR_OK)
       {
          ulLnegth  = 0 ;
          memset(TCP_Client_Recv, 0, TCP_CLIENT_RX_BUFSIZE);
          for(q = recvbuf->p ;q != NULL ;q = q->next)
          {
            if(q->len >(TCP_CLIENT_RX_BUFSIZE - ulLnegth))
            {
               memcpy(TCP_Client_Recv + ulLnegth, q->payload,(TCP_CLIENT_RX_BUFSIZE - ulLnegth));
            }
            else
            {
               memcpy(TCP_Client_Recv + ulLnegth,q->payload,q->len);
            }

            ulLnegth += q->len;
            if(ulLnegth > TCP_CLIENT_RX_BUFSIZE)break;
          }
          //uart_send_buf(uart1_t, TCP_Client_Recv, ulLnegth);
          //mbs_rtu_poll(TCP_Client_Recv, ulLnegth);
          ulLnegth = 0;
          netbuf_delete(recvbuf);

          /*清空心跳和上报时间*/
          device_info.run_params.head_send_time = 0;
          device_info.run_params.rective_wait = 0;

        }
        else if(recv_err == ERR_CLSD)
        {
          //uart_send_buf( uart1_t, "连接断开\r\n", 10 );
          netconn_close(connect);
          netconn_delete(connect);
          break;
        }
        else if(recv_err == ERR_ABRT)
        {
              //uart_send_buf( uart1_t, "出现异常\r\n", 10);
              netconn_close(connect);
              netconn_delete(connect);
              //vTaskDelete(NULL);
              break;
        }
        else if(recv_err != ERR_TIMEOUT )
        {
          // uart_send_buf( uart1_t, "读取超时\r\n", 10);
           netconn_close(connect);
           netconn_delete(connect);
           break;
        }
       if( device_info.run_params.head_send_time >= atoi(device_info.save_params.heart_time) )
       {
         device_info.run_params.head_send_time = 0;
         msg_send((uint8_t *)device_info.save_params.heart_pack, strlen(device_info.save_params.heart_pack), CLNT_MODE);
       }
       if(device_info.run_params.rective_wait >= 240)
       {
          device_info.run_params.head_send_time = 0;
          device_info.run_params.rective_wait = 0;
          //uart_send_buf( uart1_t, "等待超时\r\n", 10);
          netconn_close(connect);
          netconn_delete(connect);
          break;
       }
      }
    }
    osDelay(100);
  }
}


void task_server_socket(void *argument)
{
  struct netconn *conn,*temp_conn;
  err_t err, accept_err;
  struct netbuf *buf;
  void *data;
  uint16_t len;

  conn = netconn_new(NETCONN_TCP);

  err = netconn_bind(conn, NULL, 502);

  netconn_listen(conn);   /*绑定conn*/
  conn->recv_timeout = 20;
  if(err == ERR_OK)
  {
    while(1)
    {
      accept_err = netconn_accept(conn, &connect);  /*获取新链接*/

        /*有新的连接进来*/
      if(accept_err == ERR_OK)
      {
    	  connect->recv_timeout = 20;
        while(1)
        {
            if( (err = netconn_recv(connect, &buf)) == ERR_OK)
            {
              device_info.run_params.rective_wait = 0;
              do
              {
                netbuf_data(buf, &data, &len);
                //mbs_tcp_poll(data, len);
              }while(netbuf_next(buf) >= 0);

              netbuf_delete(buf);
            }
            else if(err == ERR_CLSD)
            {
              //uart_send_buf( uart1_t, "服务端断开\r\n", 10 );
              netconn_close(connect);
              netconn_delete(connect);
              break;
            }
            else if(err == ERR_ABRT)
            {
              //uart_send_buf( uart1_t, "出现异常\r\n", 10);
              netconn_close(connect);
              netconn_delete(connect);
              break;
            }
            else if(err != ERR_TIMEOUT )
            {
               //uart_send_buf( uart1_t, "读取超时\r\n", 10);
               netconn_close(connect);
               netconn_delete(connect);
               break;
            }
           if(device_info.run_params.rective_wait >= 240)
           {
              device_info.run_params.head_send_time = 0;
              device_info.run_params.rective_wait = 0;
              //uart_send_buf( uart1_t, "等待超时\r\n", 10);
              netconn_close(connect);
              netconn_delete(connect);
              break;
           }
           if((accept_err = netconn_accept(conn, &temp_conn)) == ERR_OK ) /*获取新链接*/
           {
             /*有新的连接进来，进入删除旧连接，改成新链接模式*/
              //uart_send_buf( uart1_t, "新的连接\r\n", 10);

              netconn_recv(connect, &buf);  /*清除缓存*/
              netbuf_delete(buf);

              netconn_close(connect);
              netconn_delete(connect);
              connect = temp_conn;
              connect->recv_timeout = 20;
           }
         }
      }
      vTaskDelay(10);

    }
  }

}

void server_recv_data(int conn)
{
  int buflen = 64;
  int ret ;
  uint8_t recv_buf[64];

  ret = read(conn, recv_buf, buflen);
  if(ret < 0)return ;

  //uart_send_buf(uart1_t, recv_buf, ret);
  close(conn);
}

