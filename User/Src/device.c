/*
 * device.c
 *
 *  Created on: 2018��5��24��
 *      Author: Snail
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "usart.h"
#include "gpio.h"
#include "device.h"
#include "string.h"
//#include "flash.h"

device_info_t device_info;
