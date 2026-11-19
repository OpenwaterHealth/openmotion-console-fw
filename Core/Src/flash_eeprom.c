/*
 * flash_eeprom.c
 *
 *  Created on: Feb 4, 2026
 *      Author: gvigelet
 */

#include "flash_eeprom.h"
#include "main.h" /* watchdog_refresh() */

/* Same bound as the HAL's private FLASH_TIMEOUT_VALUE (stm32h7xx_hal_flash.c). */
#define FLASH_ERASE_TIMEOUT_MS 50000U


/**
  * @brief  Gets the sector of a given address
  * @param  Address Address of the FLASH Memory
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if (Address < FLASH_BANK2_BASE)
    {
        sector = (Address - FLASH_BANK1_BASE) / FLASH_SECTOR_SIZE;
    }
    else
    {
        sector = (Address - FLASH_BANK2_BASE) / FLASH_SECTOR_SIZE;
    }

    if (sector > FLASH_SECTOR_7)
        sector = FLASH_SECTOR_7;

    return sector;
}

/* Function to write data to Flash */
__attribute__((section(".ramfunc")))
HAL_StatusTypeDef Flash_Write(uint32_t address, const uint32_t *data, uint32_t size_words)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t flash_word[8];

    HAL_FLASH_Unlock();
    SCB_DisableICache();

    uint32_t curr_addr = address;
    uint32_t idx = 0;

    while (idx < size_words)
    {
        uint32_t aligned_addr = curr_addr & ~0x1F;
        uint32_t word_offset = (curr_addr - aligned_addr) / 4;

        /* Read existing flashword */
        for (uint32_t i = 0; i < 8; i++)
        {
            flash_word[i] = *(__IO uint32_t *)(aligned_addr + i * 4);
        }

        /* Merge new data */
        while (word_offset < 8 && idx < size_words)
        {
            flash_word[word_offset++] = data[idx++];
            curr_addr += 4;
        }

        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            aligned_addr,
            (uint32_t)flash_word
        );

        if (status != HAL_OK)
            break;
    }

    HAL_FLASH_Lock();
    SCB_EnableICache();

    return status;
}

/* Function to read data from Flash */
HAL_StatusTypeDef Flash_Read(uint32_t address, uint32_t* data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        data[i] = *(__IO uint32_t*)(address + (i * sizeof(uint32_t)));
    }
    return HAL_OK;
}


__attribute__((section(".ramfunc")))
HAL_StatusTypeDef Flash_Write_Bytes(uint32_t address,
                                    const uint8_t *data,
                                    uint32_t size_bytes)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t flash_word[8];   // 256-bit buffer (8 x 32-bit)

    HAL_FLASH_Unlock();
    SCB_DisableICache();

    uint32_t curr_addr = address;
    uint32_t idx = 0;

    while (idx < size_bytes)
    {
        uint32_t aligned_addr = curr_addr & ~0x1F; // 32-byte alignment
        uint32_t byte_offset  = curr_addr - aligned_addr;

        /* Read existing flashword */
        for (uint32_t i = 0; i < 8; i++)
        {
            flash_word[i] = *(__IO uint32_t *)(aligned_addr + i * 4);
        }

        /* Modify bytes inside the flashword */
        uint8_t *fw_bytes = (uint8_t *)flash_word;

        while (byte_offset < 32 && idx < size_bytes)
        {
            fw_bytes[byte_offset++] = data[idx++];
            curr_addr++;
        }

        /* If we didn’t fill the entire flashword, pad with 0x00 */
        while (byte_offset < 32)
        {
            fw_bytes[byte_offset++] = 0x00;
        }

        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            aligned_addr,
            (uint32_t)flash_word
        );

        if (status != HAL_OK)
            break;
    }

    HAL_FLASH_Lock();
    SCB_EnableICache();

    return status;
}


/* Function to read data from Flash */
HAL_StatusTypeDef Flash_Read_Bytes(uint32_t address,
                                   uint8_t *data,
                                   uint32_t size_bytes)
{
    for (uint32_t i = 0; i < size_bytes; i++)
    {
        data[i] = *(__IO uint8_t *)(address + i);
    }

    return HAL_OK;
}

/* Function to erase Flash memory */
HAL_StatusTypeDef Flash_Erase(uint32_t start_address, uint32_t end_address)
{
    HAL_StatusTypeDef status;

    uint32_t first_sector = GetSector(start_address);
    uint32_t last_sector  = GetSector(end_address);
    uint32_t bank;

    if (start_address < FLASH_BANK2_BASE)
        bank = FLASH_BANK_1;
    else
        bank = FLASH_BANK_2;

    HAL_FLASH_Unlock();
    SCB_DisableICache();

    /* The same sequence as HAL_FLASHEx_Erase() for sector erase, except that
     * the wait for each sector is split into short steps with a watchdog
     * refresh between them: one 128 KB sector takes about 1 s, four times
     * the 250 ms IWDG window (CFW-01). The sector being erased is in the
     * other bank from the running code, so the CPU keeps executing. */
    pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;
    status = FLASH_WaitForLastOperation(FLASH_ERASE_TIMEOUT_MS, bank);
    for (uint32_t sector = first_sector; status == HAL_OK && sector <= last_sector; sector++) {
        FLASH_Erase_Sector(sector, bank, FLASH_VOLTAGE_RANGE_3);

        uint32_t start = HAL_GetTick();
        do {
            watchdog_refresh();
            status = FLASH_WaitForLastOperation(10U, bank);
        } while (status == HAL_TIMEOUT && (HAL_GetTick() - start) < FLASH_ERASE_TIMEOUT_MS);

        /* Erase done (or failed): clear the sector-erase request, as the HAL does. */
        if (bank == FLASH_BANK_1) {
            FLASH->CR1 &= (~(FLASH_CR_SER | FLASH_CR_SNB));
        } else {
            FLASH->CR2 &= (~(FLASH_CR_SER | FLASH_CR_SNB));
        }
    }

    HAL_FLASH_Lock();
    SCB_EnableICache();

    return status;
}
