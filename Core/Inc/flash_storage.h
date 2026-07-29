#ifndef __FLASH_STORAGE_H__
#define __FLASH_STORAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ================================================================
 *  Flash parameter storage  (§10 in 设计方案)
 *
 *  STM32F103CBT6: 128 KB Flash, 1 KB/page, pages 0~127
 *  We use the LAST page (page 127, 0x0801FC00 – 0x0801FFFF)
 *  as a dedicated parameter-storage area.
 * ================================================================ */

/* ---- Flash page address ---- */
#define FLASH_STORAGE_PAGE     127
#define FLASH_STORAGE_ADDR     0x08010000U
#define FLASH_STORAGE_SIZE     1024U

/* ---- Magic number to detect valid stored config ---- */
#define FLASH_MAGIC            0xA5C37F1EU

/* ---- Layout offsets within the 1 KB page ---- */
#define FLASH_OFF_MAC          0x000   /*  6 bytes  */
#define FLASH_OFF_IP           0x008   /*  4 bytes  */
#define FLASH_OFF_SUBNET       0x00C   /*  4 bytes  */
#define FLASH_OFF_GATEWAY      0x010   /*  4 bytes  */
#define FLASH_OFF_MATRIX       0x014   /* 36×4=144 bytes (6×6 float32) */
#define FLASH_OFF_ZERO         0x0A4   /*  6×4= 24 bytes (6× float32)  */
#define FLASH_OFF_FW_VERSION   0x0BC   /*  2 bytes  */
#define FLASH_OFF_MAGIC        0x0C0   /*  4 bytes  */
#define FLASH_OFF_CRC          0x0C4   /*  2 bytes  */
/* total: 0xC6 = 198 bytes, well within 1 KB */

/* ================================================================
 *  Public API
 * ================================================================ */

/**
 * Load all parameters from flash into RAM (g_config, g_matrix, g_sensor.force_zero).
 * Returns 1 if valid data found and loaded, 0 if flash is empty/corrupt
 * (caller should keep current RAM defaults on failure).
 */
int flash_load_all(void);

/**
 * Save network config (MAC, IP, subnet, gateway, FW version) to flash.
 * Returns 1 on success, 0 on failure.
 */
int flash_save_config(void);

/**
 * Save 6×6 decoupling matrix to flash.
 * Returns 1 on success, 0 on failure.
 */
int flash_save_matrix(void);

/**
 * Save 6-axis zero offsets (force_zero[6]) to flash.
 * Returns 1 on success, 0 on failure.
 */
int flash_save_zero(void);

/**
 * Erase the entire storage page (used before rewrite-all).
 */
void flash_erase_storage(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_STORAGE_H__ */
