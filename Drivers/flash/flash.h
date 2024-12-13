#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "main.h"  

//=========================数据类型宏定义
#define __IO    volatile 
typedef __IO uint16_t vu16;
 
//=========================用户根据自己的需要设置
#define STM32_FLASH_SIZE 	64 	 	//所选STM32的FLASH容量大小(单位为K)

#if     STM32_FLASH_SIZE < 256      //设置扇区大小
    #define STM_SECTOR_SIZE     1024    //1K字节   //////////////
#else 
    #define STM_SECTOR_SIZE	    2048    //2K字节
#endif	

#define STM32_FLASH_BASE    0x08000000 		//STM32 FLASH的起始地址
#define FLASH_SAVE_ADDR     STM32_FLASH_BASE + STM_SECTOR_SIZE*62	//写Flash的地址，这里从倒数第二页开始
#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
#define FLASH_WAITETIME  	50000          	//FLASH等待超时时间
 
 
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);		  //读出半字  
void STMFLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead);   		//从指定地址开始读出指定长度的数据
void Flash_PageErase(uint32_t PageAddress);     //扇区擦除
						   
#endif
 