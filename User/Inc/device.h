/*
 * device.h
 *
 *  Created on: 2018Äê5ÔÂ24ÈÕ
 *      Author: Snail
 */

#ifndef INC_DEVICE_H_
#define INC_DEVICE_H_

typedef struct
{
  char local_ip[16];
  char local_gateway[16];
  char local_mask[16];
  char local_mac[20];
  char local_dns_addr[16];

  char serial_num[64];
  char server_addr[24];
  char server_port[8];
  char heart_pack[8];
  char heart_time[8];
  uint16_t dhcp_en;
  uint16_t device_mod;
}save_param_t;


typedef struct
{
  uint8_t debug_en;
  uint8_t create_tcp;
  uint16_t rective_wait;
  uint16_t head_send_time;
}run_param_t;

typedef struct
{
  save_param_t save_params;
  run_param_t  run_params;
}device_info_t;

#endif /* INC_DEVICE_H_ */
