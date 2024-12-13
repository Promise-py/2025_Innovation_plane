#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "main.h"  

//=========================�������ͺ궨��
#define __IO    volatile 
typedef __IO uint16_t vu16;
 
//=========================�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 	64 	 	//��ѡSTM32��FLASH������С(��λΪK)

#if     STM32_FLASH_SIZE < 256      //����������С
    #define STM_SECTOR_SIZE     1024    //1K�ֽ�   //////////////
#else 
    #define STM_SECTOR_SIZE	    2048    //2K�ֽ�
#endif	

#define STM32_FLASH_BASE    0x08000000 		//STM32 FLASH����ʼ��ַ
#define FLASH_SAVE_ADDR     STM32_FLASH_BASE + STM_SECTOR_SIZE*62	//дFlash�ĵ�ַ������ӵ����ڶ�ҳ��ʼ
#define STM32_FLASH_WREN 	1              	//ʹ��FLASHд��(0��������;1��ʹ��)
#define FLASH_WAITETIME  	50000          	//FLASH�ȴ���ʱʱ��
 
 
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);		  //��������  
void STMFLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(uint32_t ReadAddr,uint16_t *pBuffer,uint16_t NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����
void Flash_PageErase(uint32_t PageAddress);     //��������
						   
#endif
 