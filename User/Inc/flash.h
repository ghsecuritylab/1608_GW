/*
 * flash.h
 *
 *  Created on: 2018年5月24日
 *      Author: Snail
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "stm32F4xx.h"

#define STM32_FLASH_BASE  0x8000000

/* 各個扇區的起始 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//扇区0起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//扇区1起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//扇区2起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//扇区3起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//扇区4起始地址, 64 Kbytes
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//扇区5起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//扇区6起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//扇区7起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//扇区8起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//扇区9起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//扇区10起始地址,128 Kbytes
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//扇区11起始地址,128 Kbytes

#define FLASH_Sector_0     ((uint16_t)0x0000) /*!< Sector Number 0   */
#define FLASH_Sector_1     ((uint16_t)0x0008) /*!< Sector Number 1   */
#define FLASH_Sector_2     ((uint16_t)0x0010) /*!< Sector Number 2   */
#define FLASH_Sector_3     ((uint16_t)0x0018) /*!< Sector Number 3   */
#define FLASH_Sector_4     ((uint16_t)0x0020) /*!< Sector Number 4   */
#define FLASH_Sector_5     ((uint16_t)0x0028) /*!< Sector Number 5   */
#define FLASH_Sector_6     ((uint16_t)0x0030) /*!< Sector Number 6   */
#define FLASH_Sector_7     ((uint16_t)0x0038) /*!< Sector Number 7   */
#define FLASH_Sector_8     ((uint16_t)0x0040) /*!< Sector Number 8   */
#define FLASH_Sector_9     ((uint16_t)0x0048) /*!< Sector Number 9   */
#define FLASH_Sector_10    ((uint16_t)0x0050) /*!< Sector Number 10  */
#define FLASH_Sector_11    ((uint16_t)0x0058) /*!< Sector Number 11  */
#define FLASH_Sector_12    ((uint16_t)0x0080) /*!< Sector Number 12  */
#define FLASH_Sector_13    ((uint16_t)0x0088) /*!< Sector Number 13  */
#define FLASH_Sector_14    ((uint16_t)0x0090) /*!< Sector Number 14  */
#define FLASH_Sector_15    ((uint16_t)0x0098) /*!< Sector Number 15  */
#define FLASH_Sector_16    ((uint16_t)0x00A0) /*!< Sector Number 16  */
#define FLASH_Sector_17    ((uint16_t)0x00A8) /*!< Sector Number 17  */
#define FLASH_Sector_18    ((uint16_t)0x00B0) /*!< Sector Number 18  */
#define FLASH_Sector_19    ((uint16_t)0x00B8) /*!< Sector Number 19  */
#define FLASH_Sector_20    ((uint16_t)0x00C0) /*!< Sector Number 20  */
#define FLASH_Sector_21    ((uint16_t)0x00C8) /*!< Sector Number 21  */
#define FLASH_Sector_22    ((uint16_t)0x00D0) /*!< Sector Number 22  */
#define FLASH_Sector_23    ((uint16_t)0x00D8) /*!< Sector Number 23  */

/** @defgroup FLASH_Flags
  * @{
  */
#define FLAG_EOP                 ((uint32_t)0x00000001)  /*!< FLASH End of Operation flag               */
#define FLAG_OPERR               ((uint32_t)0x00000002)  /*!< FLASH operation Error flag                */
#define FLAG_WRPERR              ((uint32_t)0x00000010)  /*!< FLASH Write protected error flag          */
#define FLAG_PGAERR              ((uint32_t)0x00000020)  /*!< FLASH Programming Alignment error flag    */
#define FLAG_PGPERR              ((uint32_t)0x00000040)  /*!< FLASH Programming Parallelism error flag  */
#define FLAG_PGSERR              ((uint32_t)0x00000080)  /*!< FLASH Programming Sequence error flag     */
#define FLAG_RDERR               ((uint32_t)0x00000100)  /*!< Read Protection error flag (PCROP)        */
#define FLAG_BSY                 ((uint32_t)0x00010000)  /*!< FLASH Busy flag                           */

typedef enum
{
  BUSY = 1,
  ERROR_RD,
  ERROR_PGS,
  ERROR_PGP,
  ERROR_PGA,
  ERROR_WRP,
  ERROR_PROGRAM,
  ERROR_OPERATION,
  FLASH_COMPLETE
}FLASH_Status;


void flash_write(uint32_t write_addr, uint32_t *write_buf, uint32_t write_size);
void flash_read(uint32_t read_addr, uint32_t *read_buf, uint32_t read_size);

#endif /* INC_FLASH_H_ */
