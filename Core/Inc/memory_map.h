/*
 * memory_map.h
 *
 *  Created on: Mar 4, 2024
 *      Author: gvigelet
 *
 *  STM32H743 internal flash: 2 MB = 2 banks x 1 MB, 16 sectors x 128 KB.
 *  The flash map is owned by the console bootloader (open-motion-console-bl,
 *  Core/Inc/memory_map.h). The MEM_* values below mirror it and must be kept
 *  in sync with it.
 *
 *   Addr range              Size   Sector         Region             Owner
 *  ---------------------------------------------------------------------------
 *  0x08000000-0x0801FFFF    128K   0  (B1 S0)     BOOTLOADER         bootloader
 *  0x08020000-0x0809FFFF    512K   1-4 (B1 S1-4)  APP SLOT           this image
 *  0x080A0000-0x080BFFFF    128K   5  (B1 S5)     ANTI-ROLLBACK FLOOR bootloader
 *  0x080C0000-0x0819FFFF    896K   6-12           RESERVED           -
 *  0x081A0000-0x081DFFFF    256K   13-14 (B2 S5-6) APPLICATION-OWNED application
 *  0x081E0000-0x081FFFFF    128K   15 (B2 S7)     USER CONFIG        application
 *  ---------------------------------------------------------------------------
 *
 *  The slot build (STM32H743XX_FLASH.ld) runs at MEM_APP_RUN_ADDRESS and must
 *  end at MEM_SLOT1_END. The application writes only USER CONFIG (motion_config).
 *  It must never write the bootloader or the anti-rollback floor. The console
 *  does not use the APPLICATION-OWNED sectors (the sensor keeps its camera FPGA
 *  bitstream there); the odometer lives in the external EEPROM (odometer.h),
 *  not in internal flash.
 *
 *  The bare-metal build (STM32H743XX_FLASH_BARE.ld) runs at 0x08000000 with no
 *  bootloader, so the bootloader regions do not exist for it.
 *
 *  Sector names below are per bank (ADDR_FLASH_SECTOR_n_BANKm, n = 0-7); the
 *  bootloader numbers sectors 0-15 across both banks.
 */

#ifndef INC_MEMORY_MAP_H_
#define INC_MEMORY_MAP_H_

#ifdef __cplusplus
 extern "C" {
#endif
 /* Base address of the Flash pages */
#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

#define ADDR_FLASH_END_ADDRESS     ((uint32_t)0x08200000)

/* ── Bootloader flash map (mirrors open-motion-console-bl memory_map.h) ───── */
#define MEM_SECTOR_SIZE           (0x00020000UL)   /* 128 KB per sector           */

/* Bootloader (sector 0). Not present in the bare-metal build. */
#define MEM_BOOTLOADER_BASE       (0x08000000UL)
#define MEM_BOOTLOADER_SIZE       (0x00020000UL)   /* 128 KB                      */

/* Active application slot (sectors 1-4). The only DFU-writable region. */
#define MEM_SLOT1_BASE            (0x08020000UL)
#define MEM_SLOT1_SIZE            (0x00080000UL)   /* 512 KB                      */
#define MEM_SLOT1_END             (MEM_SLOT1_BASE + MEM_SLOT1_SIZE) /* 0x080A0000 */

/* Signed-image header at the start of the slot; the image runs after it.
   Must match SFU_IMG_IMAGE_OFFSET in the bootloader, the slot linker script
   (STM32H743XX_FLASH.ld) and VECT_TAB_BASE_ADDRESS in system_stm32h7xx.c. */
#define MEM_APP_IMAGE_OFFSET      (0x00000400UL)   /* 1 KB                        */
#define MEM_APP_RUN_ADDRESS       (MEM_SLOT1_BASE + MEM_APP_IMAGE_OFFSET) /* 0x08020400 */

/* Anti-rollback floor (sector 5). Erased and rewritten by the bootloader only;
   the application must never write it. */
#define MEM_ANTIROLLBACK_BASE     (0x080A0000UL)
#define MEM_ANTIROLLBACK_SIZE     (0x00020000UL)   /* 128 KB                      */

/* Reserved for future bootloader use (sectors 6-12). */
#define MEM_RESERVED_BASE         (0x080C0000UL)
#define MEM_RESERVED_SIZE         (0x000E0000UL)   /* 896 KB                      */

/* Application-owned (sectors 13-14). The bootloader never touches it; unused
   on the console. */
#define MEM_APP_OWNED_BASE        (0x081A0000UL)
#define MEM_APP_OWNED_SIZE        (0x00040000UL)   /* 256 KB                      */

/* User configuration (sector 15): motion_config / calibration JSON.
   Read-only over DFU; written by the application. */
#define MEM_USER_CONFIG_BASE      (0x081E0000UL)
#define MEM_USER_CONFIG_SIZE      (0x00020000UL)   /* 128 KB                      */

/* Persistent data in internal flash. Each tenant needs a whole 128 KB sector
 * (the minimum erase granularity). */
#define FLASH_USER_START_ADDR     ADDR_FLASH_SECTOR_7_BANK2  /* motion_config / calibration */

#ifndef __cplusplus
_Static_assert(ADDR_FLASH_SECTOR_7_BANK2 == MEM_USER_CONFIG_BASE,
               "user config sector disagrees with the bootloader flash map");
_Static_assert(ADDR_FLASH_SECTOR_5_BANK1 == MEM_ANTIROLLBACK_BASE,
               "anti-rollback floor disagrees with the bootloader flash map");
#endif

#ifdef __cplusplus
}
#endif

#endif /* INC_MEMORY_MAP_H_ */
