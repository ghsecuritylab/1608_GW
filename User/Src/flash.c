/*
 * flash.c
 *
 *  Created on: 2018年5月24日
 *      Author: Snail
 */
#include <stdint.h>
#include "flash.h"


/* 私有函數  -----------------------------------------------*/
/**
  * @brief  解鎖FLASH控制寄存器
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
}
/**
  * @brief  鎖定FLASH控制寄存器
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
  FLASH->CR |= FLASH_CR_LOCK;
}
/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_RD, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_GetStatus(void)
{
	  FLASH_Status flashstatus = FLASH_COMPLETE;

	  if((FLASH->SR & FLAG_BSY) == FLASH_FLAG_BSY)
	  {
	    flashstatus = BUSY;
	  }
	  else
	  {
	    if((FLASH->SR & FLAG_WRPERR) != (uint32_t)0x00)
	    {
	      flashstatus = ERROR_WRP;
	    }
	    else
	    {
	      if((FLASH->SR & FLAG_RDERR) != (uint32_t)0x00)
	      {
	        flashstatus = ERROR_RD;
	      }
	      else
	      {
	        if((FLASH->SR & (uint32_t)0xEF) != (uint32_t)0x00)
	        {
	          flashstatus = ERROR_PROGRAM;
	        }
	        else
	        {
	          if((FLASH->SR & FLASH_FLAG_OPERR) != (uint32_t)0x00)
	          {
	            flashstatus = ERROR_OPERATION;
	          }
	          else
	          {
	            flashstatus = FLASH_COMPLETE;
	          }
	        }
	      }
	    }
	  }
  /* Return the FLASH Status */
  return flashstatus;
}
/**
  * @brief  等待FLASH操作完成
  * @param  None
  * @retval FLASH Status: The returned value can be: BUSY, ERROR_PROGRAM,
  *                       ERROR_WRP, ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status flash_wait_op(void)
{
  __IO FLASH_Status status = HAL_FLASH_ERROR_NONE;

  /* Check for the FLASH Status */
  status = FLASH_GetStatus();

  /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
  while(status == BUSY)
  {
    status = FLASH_GetStatus();
  }
  /* Return the operation status */
  return status;
}

/**
  * @brief  启用或禁用数据缓存功能。
  * @param   NewState:数据缓存的新状态。
  *          这个参数可以是:ENABLE/DISABLE。
  * @retval None
  */
void FLASH_DataCacheCmd(FunctionalState NewState)
{
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_DCEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_DCEN);
  }
}
/**
  * @brief  清除FLASH扇區(從標準庫移植過來)
  *
  * @note   If an erase and a program operations are requested simustaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  FLASH_Sector: The Sector number to be erased.
  *
  *  @note  For STM32F405xx/407xx and STM32F415xx/417xx devices this parameter can
  *         be a value between FLASH_Sector_0 and FLASH_Sector_11.
  *
  *         For STM32F42xxx/43xxx devices this parameter can be a value between
  *         FLASH_Sector_0 and FLASH_Sector_23.
  *
  *         For STM32F401xx devices this parameter can be a value between
  *         FLASH_Sector_0 and FLASH_Sector_5.
  *
  *         For STM32F411xE devices this parameter can be a value between
  *         FLASH_Sector_0 and FLASH_Sector_7.
  *
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.
  *          This parameter can be one of the following values:
  *            @arg VoltageRange_1: when the device voltage range is 1.8V to 2.1V,
  *                                  the operation will be done by byte (8-bit)
  *            @arg VoltageRange_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VoltageRange_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VoltageRange_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
  *                                  the operation will be done by double word (64-bit)
  *
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0x0;
  FLASH_Status status = FLASH_COMPLETE;

  assert_param(IS_FLASH_SECTOR(FLASH_Sector));
  assert_param(IS_VOLTAGERANGE(VoltageRange));

  if(VoltageRange == FLASH_VOLTAGE_RANGE_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == FLASH_VOLTAGE_RANGE_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == FLASH_VOLTAGE_RANGE_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }

  status = flash_wait_op();

  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase the sector */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= tmp_psize;
    FLASH->CR &= 0xFFFFFF07;
    FLASH->CR |= FLASH_CR_SER | FLASH_Sector;
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait for last operation to be completed */
    status = flash_wait_op();

    /* if the erase operation is completed, disable the SER Bit */
    FLASH->CR &= (~FLASH_CR_SER);
    FLASH->CR &= 0xFFFFFF07;
  }
  return status;
}

/**
  * @brief  寫入一个字(32位)在指定的地址。
  *
  * @note   This function must be used when the device voltage range is from 2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simustaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  *         This parameter can be any address in Program memory zone or in OTP zone.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = HAL_FLASH_ERROR_NONE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = flash_wait_op();

  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_WORD;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint32_t*)Address = Data;

    /* Wait for last operation to be completed */
    status = flash_wait_op();

    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  }
  /* Return the Program Status */
  return status;
}
uint32_t flash_read_word(uint32_t read_addr)
{
	return  *(volatile uint32_t *)read_addr;
}

uint16_t flash_get_sector(uint32_t addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)
		return FLASH_Sector_0;
	else if(addr < ADDR_FLASH_SECTOR_2)
		return FLASH_Sector_1;
	else if(addr < ADDR_FLASH_SECTOR_3)
		return FLASH_Sector_2;
	else if(addr < ADDR_FLASH_SECTOR_4)
		return FLASH_Sector_3;
	else if(addr < ADDR_FLASH_SECTOR_5)
		return FLASH_Sector_4;
	else if(addr < ADDR_FLASH_SECTOR_6)
		return FLASH_Sector_5;
	else if(addr < ADDR_FLASH_SECTOR_7)
		return FLASH_Sector_6;
	else if(addr < ADDR_FLASH_SECTOR_8)
		return FLASH_Sector_7;
	else if(addr < ADDR_FLASH_SECTOR_9)
		return FLASH_Sector_8;
	else if(addr < ADDR_FLASH_SECTOR_10)
		return FLASH_Sector_9;
	else if(addr < ADDR_FLASH_SECTOR_11)
		return FLASH_Sector_10;
	return FLASH_Sector_11;
}

void flash_write(uint32_t write_addr, uint32_t *write_buf, uint32_t write_size)
{
	uint32_t addrx = 0;
	uint32_t endaddr = 0;

	if(write_addr < STM32_FLASH_BASE || write_addr % 4)
		return ;
	FLASH_Unlock();
	FLASH_DataCacheCmd(DISABLE);

	endaddr = write_addr + write_size * 4;

	if(addrx < 0X1FFF0000)
	{
		if(FLASH_EraseSector(flash_get_sector(write_addr),FLASH_VOLTAGE_RANGE_3)!= FLASH_COMPLETE)//VCC=2.7~3.6V之间!!
		{
		}
	}

	while(write_addr < endaddr)
	{
		if(FLASH_ProgramWord(write_addr,*write_buf) != FLASH_COMPLETE)
		{
			break;
		}
		write_addr += 4;
		write_buf ++;
	}
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
}

void flash_read(uint32_t read_addr, uint32_t *read_buf, uint32_t read_size)
{
	uint32_t read_pos = 0;
	while(read_pos < read_size)
	{
		read_buf[read_pos] = flash_read_word(read_addr);
		read_pos++;
		read_addr += 4;
	}
}
