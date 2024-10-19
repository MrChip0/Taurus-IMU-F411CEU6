#include "bsp_flash.h"
#include "data_processing.h"
#include "string.h"
#include "stm32f4xx_hal.h"

void Flash_Write_Float(uint32_t address, float data) 
{
    uint32_t dataToWrite = *(uint32_t *)&data;
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t sectorError = 0;
    if (address >= 0x08000000 && address < 0x08004000) {
        eraseInitStruct.Sector = FLASH_SECTOR_0;
    } else if (address >= 0x08004000 && address < 0x08008000) {
        eraseInitStruct.Sector = FLASH_SECTOR_1;
    } else if (address >= 0x08008000 && address < 0x0800C000) {
        eraseInitStruct.Sector = FLASH_SECTOR_2;
    } else if (address >= 0x0800C000 && address < 0x08010000) {
        eraseInitStruct.Sector = FLASH_SECTOR_3;
    } else if (address >= 0x08010000 && address < 0x08020000) {
        eraseInitStruct.Sector = FLASH_SECTOR_4;
    } else if (address >= 0x08020000 && address < 0x08040000) {
        eraseInitStruct.Sector = FLASH_SECTOR_5;
    } else if (address >= 0x08040000 && address < 0x08080000) {
        eraseInitStruct.Sector = FLASH_SECTOR_6;
    } else if (address >= 0x08080000 && address < 0x080C0000) {
        eraseInitStruct.Sector = FLASH_SECTOR_7;
    } else {
        // Error_Handler();
        return;
    }
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; 
    eraseInitStruct.NbSectors = 1;
    if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK) {
        // Error_Handler();
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, dataToWrite) != HAL_OK) {
        // Error_Handler();
    }
    HAL_FLASH_Lock();
}

// Flash读取float数值函数
float Flash_Read_Float(uint32_t address)
{
    uint32_t dataRead = *(uint32_t *)address;
    float floatDataRead = *(float *)&dataRead;
    return floatDataRead;
}



