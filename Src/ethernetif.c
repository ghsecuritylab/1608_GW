/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "lwip/opt.h"

#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include <string.h>
#include "cmsis_os.h"
#include "device.h"
# include <stdio.h>
# include <stdlib.h>

/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT ( portMAX_DELAY )
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE ( 350 )
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'


/* 私有變量 ---------------------------------------------------------*/
__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */

__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */

__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */

__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */

/* Semaphore to signal incoming packets */
osSemaphoreId s_xSemaphore = NULL;
/* Global Ethernet handle */
ETH_HandleTypeDef heth;

extern device_info_t device_info;

/* 私有函数 ---------------------------------------------------------*/

void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(ethHandle->Instance==ETH)
  {
    /* Enable Peripheral clock */
    __HAL_RCC_ETH_CLK_ENABLE();
  
    /**ETH GPIO 配置
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 外設中斷初始化 */
    HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
  }
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef* ethHandle)
{
  if(ethHandle->Instance==ETH)
  {
    /* 外設關閉 */
    __HAL_RCC_ETH_CLK_DISABLE();
  
    /**ETH GPIO Configuration    
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13);

    /* 外設終端恢復*/
    HAL_NVIC_DisableIRQ(ETH_IRQn);

  }
}

/**
  * @brief  以太网Rx传输完成回调
  * @param  heth: ETH句柄
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  osSemaphoreRelease(s_xSemaphore);
}
/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH) 
*******************************************************************************/
/**
 * 在這個函數裏面硬件應該被初始化
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{ 
  uint32_t regvalue = 0;
  HAL_StatusTypeDef hal_eth_init_status;

  uint8_t MACAddr[6] ;
  uint32_t temp;
  
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;

  temp = *(volatile uint32_t*)(0x1FFFF7E8);/*获取stm32的唯一DI前24位作为mac地址*/


  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = (temp>>16)&0XFF;
  MACAddr[4] = (temp>>8)&0XFFF;
  MACAddr[5] = temp&0XFF;

  itoa(MACAddr[0], (char*)&device_info.save_params.local_mac[0], 10);
  device_info.save_params.local_mac[2] = '-';
  itoa(MACAddr[1], (char*)&device_info.save_params.local_mac[3], 10);
  device_info.save_params.local_mac[5] = '-';
  itoa(MACAddr[2], (char*)&device_info.save_params.local_mac[6], 10);
  device_info.save_params.local_mac[8] = '-';
  itoa(MACAddr[3], (char*)&device_info.save_params.local_mac[9],10);
  device_info.save_params.local_mac[11] = '-';
  itoa(MACAddr[0], (char*)&device_info.save_params.local_mac[12], 10);
  device_info.save_params.local_mac[14] = '-';
  itoa(MACAddr[5], (char*)&device_info.save_params.local_mac[15], 10);

  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  hal_eth_init_status = HAL_ETH_Init(&heth);

  if (hal_eth_init_status == HAL_OK)
  {
    netif->flags |= NETIF_FLAG_LINK_UP;
  }
  HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
     
  HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
 
#if LWIP_ARP || LWIP_ETHERNET 

  /* 設置MAC地址 */
  netif->hwaddr_len = ETH_HWADDR_LEN;
  /* set MAC hardware address */
  netif->hwaddr[0] =  heth.Init.MACAddr[0];
  netif->hwaddr[1] =  heth.Init.MACAddr[1];
  netif->hwaddr[2] =  heth.Init.MACAddr[2];
  netif->hwaddr[3] =  heth.Init.MACAddr[3];
  netif->hwaddr[4] =  heth.Init.MACAddr[4];
  netif->hwaddr[5] =  heth.Init.MACAddr[5];
  
  /* maximum transfer unit */
  netif->mtu = 1500;
  
  /* Accept broadcast address and ARP traffic */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  #if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
  #else 
    netif->flags |= NETIF_FLAG_BROADCAST;
  #endif /* LWIP_ARP */
  
/* create a binary semaphore used for informing ethernetif of frame reception */
  osSemaphoreDef(SEM);
  s_xSemaphore = osSemaphoreCreate(osSemaphore(SEM) , 1 );

/* create the task that handles the ETH_MAC */
  osThreadDef(EthIf, ethernetif_input, osPriorityRealtime, 0, INTERFACE_THREAD_STACK_SIZE);
  osThreadCreate (osThread(EthIf), netif);
  /* Enable MAC and DMA transmission and reception */
  HAL_ETH_Start(&heth);
  /* Read Register Configuration */
  HAL_ETH_ReadPHYRegister(&heth, PHY_ISFR, &regvalue);
  regvalue |= (PHY_ISFR_INT4);

  /* Enable Interrupt on change of link status */ 
  HAL_ETH_WritePHYRegister(&heth, PHY_ISFR , regvalue );
  
  /* Read Register Configuration */
  HAL_ETH_ReadPHYRegister(&heth, PHY_ISFR , &regvalue);

#endif /* LWIP_ARP || LWIP_ETHERNET */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  err_t errval;
  struct pbuf *q;
  uint8_t *buffer = (uint8_t *)(heth.TxDesc->Buffer1Addr);
  __IO ETH_DMADescTypeDef *DmaTxDesc;
  uint32_t framelength = 0;
  uint32_t bufferoffset = 0;
  uint32_t byteslefttocopy = 0;
  uint32_t payloadoffset = 0;
  DmaTxDesc = heth.TxDesc;
  bufferoffset = 0;
  
  /* copy frame from pbufs to driver buffers */
  for(q = p; q != NULL; q = q->next)
    {
      /* Is this buffer available? If not, goto error */
      if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
      {
        errval = ERR_USE;
        goto error;
      }
    
      /* Get bytes in current lwIP buffer */
      byteslefttocopy = q->len;
      payloadoffset = 0;
    
      /* Check if the length of data to copy is bigger than Tx buffer size*/
      while( (byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE )
      {
        /* Copy data to Tx buffer*/
        memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)q->payload + payloadoffset), (ETH_TX_BUF_SIZE - bufferoffset) );
      
        /* Point to next descriptor */
        DmaTxDesc = (ETH_DMADescTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);
      
        /* Check if the buffer is available */
        if((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t)RESET)
        {
          errval = ERR_USE;
          goto error;
        }
      
        buffer = (uint8_t *)(DmaTxDesc->Buffer1Addr);
      
        byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
        payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
        framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
        bufferoffset = 0;
      }
    
      /* Copy the remaining bytes */
      memcpy( (uint8_t*)((uint8_t*)buffer + bufferoffset), (uint8_t*)((uint8_t*)q->payload + payloadoffset), byteslefttocopy );
      bufferoffset = bufferoffset + byteslefttocopy;
      framelength = framelength + byteslefttocopy;
    }
  
  /* Prepare transmit descriptors to give to DMA */ 
  HAL_ETH_TransmitFrame(&heth, framelength);
  
  errval = ERR_OK;
  
error:
  
  /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
  if ((heth.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t)RESET)
  {
    /* Clear TUS ETHERNET DMA flag */
    heth.Instance->DMASR = ETH_DMASR_TUS;

    /* Resume DMA transmission*/
    heth.Instance->DMATPDR = 0;
  }
  return errval;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
   */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;
  struct pbuf *q = NULL;
  uint16_t len = 0;
  uint8_t *buffer;
  __IO ETH_DMADescTypeDef *dmarxdesc;
  uint32_t bufferoffset = 0;
  uint32_t payloadoffset = 0;
  uint32_t byteslefttocopy = 0;
  uint32_t i=0;
  

  /* get received frame */
  if (HAL_ETH_GetReceivedFrame_IT(&heth) != HAL_OK)
    return NULL;
  
  /* Obtain the size of the packet and put it into the "len" variable. */
  len = heth.RxFrameInfos.length;
  buffer = (uint8_t *)heth.RxFrameInfos.buffer;
  
  if (len > 0)
  {
    /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  }
  
  if (p != NULL)
  {
    dmarxdesc = heth.RxFrameInfos.FSRxDesc;
    bufferoffset = 0;
    for(q = p; q != NULL; q = q->next)
    {
      byteslefttocopy = q->len;
      payloadoffset = 0;
      
      /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size*/
      while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE )
      {
        /* Copy data to pbuf */
        memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));
        
        /* Point to next descriptor */
        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
        buffer = (uint8_t *)(dmarxdesc->Buffer1Addr);
        
        byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
        payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
        bufferoffset = 0;
      }
      /* Copy remaining data in pbuf */
      memcpy( (uint8_t*)((uint8_t*)q->payload + payloadoffset), (uint8_t*)((uint8_t*)buffer + bufferoffset), byteslefttocopy);
      bufferoffset = bufferoffset + byteslefttocopy;
    }
  }  
  
    /* Release descriptors to DMA */
    /* Point to first descriptor */
    dmarxdesc = heth.RxFrameInfos.FSRxDesc;
    /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
    for (i=0; i< heth.RxFrameInfos.SegCount; i++)
    {  
      dmarxdesc->Status |= ETH_DMARXDESC_OWN;
      dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
    }
    
    /* Clear Segment_Count */
    heth.RxFrameInfos.SegCount =0;  
  
  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((heth.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)  
  {
    /* Clear RBUS ETHERNET DMA flag */
    heth.Instance->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    heth.Instance->DMARPDR = 0;
  }
  return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input( void const * argument ) 
{
  struct pbuf *p;
  struct netif *netif = (struct netif *) argument;
  
  for( ;; )
  {
    if (osSemaphoreWait( s_xSemaphore, TIME_WAITING_FOR_INPUT)==osOK)
    {
      do
      {   
        p = low_level_input( netif );
        if   (p != NULL)
        {
          if (netif->input( p, netif) != ERR_OK )
          {
            pbuf_free(p);
          }
        }
      } while(p!=NULL);
    }
  }
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{  
  err_t errval;
  errval = ERR_OK;
    
/* USER CODE BEGIN 5 */ 
    
/* USER CODE END 5 */  
    
  return errval;
  
}
#endif /* LWIP_ARP */ 

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));
  
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
  netif->output = etharp_output;
#else
  /* The user should write ist own code in low_level_output_arp_off function */
  netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */
 
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/* USER CODE BEGIN 6 */

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_jiffies(void)
{
  return HAL_GetTick();
}

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/* USER CODE END 6 */

/**
  * @brief  This function sets the netif link status.
  * @param  netif: the network interface
  * @retval None
  */  
void ethernetif_set_link(void const *argument)
{
  uint32_t regvalue = 0;
  struct link_str *link_arg = (struct link_str *)argument;
  
  for(;;)
  {
    /* Read PHY_BSR*/
    HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);
    
    regvalue &= PHY_LINKED_STATUS;
    
    /* Check whether the netif link down and the PHY link is up */
    if(!netif_is_link_up(link_arg->netif) && (regvalue))
    {
      /* network cable is connected */ 
      netif_set_link_up(link_arg->netif);        
    }
    else if(netif_is_link_up(link_arg->netif) && (!regvalue))
    {
      /* network cable is dis-connected */
      netif_set_link_down(link_arg->netif);
    }
    
    /* Suspend thread for 200 ms */
    osDelay(200);
  }
}

/* USER CODE BEGIN 7 */

/* USER CODE END 7 */

#if LWIP_NETIF_LINK_CALLBACK
/**
  * @brief  Link callback function, this function is called on change of link status
  *         to update low level driver configuration.
* @param  netif: The network interface
  * @retval None
  */
void ethernetif_update_config(struct netif *netif)
{
  __IO uint32_t tickstart = 0;
  uint32_t regvalue = 0;
  
  if(netif_is_link_up(netif))
  { 
	  /* 重新启动自动协商 */
    if(heth.Init.AutoNegotiation != ETH_AUTONEGOTIATION_DISABLE)
    {
      /* 启动自动协商 */
      HAL_ETH_WritePHYRegister(&heth, PHY_BCR, PHY_AUTONEGOTIATION);
      
      /* Get tick */
      tickstart = HAL_GetTick();
      
      /* 等到自动协商完成 */
      do
      {
        HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);

        if((HAL_GetTick() - tickstart ) > 1000)
        {     
          goto error;
        }   
      } while (((regvalue & PHY_AUTONEGO_COMPLETE) != PHY_AUTONEGO_COMPLETE));
      
      /* 讀取自動協商的結果 */
      HAL_ETH_ReadPHYRegister(&heth, PHY_SR, &regvalue);
      
      if((regvalue & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
      {
        heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;  
      }
      else
      {
        heth.Init.DuplexMode = ETH_MODE_HALFDUPLEX;           
      }
      if(regvalue & PHY_SPEED_STATUS)
      {  
        /* Set Ethernet speed to 10M following the auto-negotiation */
        heth.Init.Speed = ETH_SPEED_10M; 
      }
      else
      {   
        heth.Init.Speed = ETH_SPEED_100M;
      }
    }
    else /* 自動協商失敗  */
    {
    error :
      assert_param(IS_ETH_SPEED(heth.Init.Speed));
      assert_param(IS_ETH_DUPLEX_MODE(heth.Init.DuplexMode));
      
      /* 将MAC速率和双工模式设置为PHY */
      HAL_ETH_WritePHYRegister(&heth, PHY_BCR, ((uint16_t)(heth.Init.DuplexMode >> 3) |
                                                     (uint16_t)(heth.Init.Speed >> 1))); 
    }

    /*以太網MAC重新配置 */
    HAL_ETH_ConfigMAC(&heth, (ETH_MACInitTypeDef *) NULL);
    HAL_ETH_Start(&heth);
    ethernetif_notify_conn_changed(netif);
  }
  else
  {
    /*網口斷開鏈接*/
    HAL_ETH_Stop(&heth);
  }

  ethernetif_notify_conn_changed(netif);
}

/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
__weak void ethernetif_notify_conn_changed(struct netif *netif)
{

}
#endif /* LWIP_NETIF_LINK_CALLBACK */
