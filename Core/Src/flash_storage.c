#include "flash_storage.h"
#include "sensor_types.h"
#include "crc.h"
#include "stm32f1xx_hal.h"
#include <string.h>

/* ================================================================
 *  Internal helpers — STM32F1 HAL Flash
 * ================================================================ */

static int flash_unlock(void)
{
    HAL_FLASH_Unlock();
    return 1;
}

static void flash_lock(void)
{
    HAL_FLASH_Lock();
}

/**
 * Erase the storage page.  Must be called with flash unlocked.
 */
void flash_erase_storage(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_STORAGE_ADDR;
    erase.NbPages     = 1;

    HAL_FLASHEx_Erase(&erase, &page_error);
}

/**
 * Program a 16-bit half-word at an absolute flash address.
 * Caller must unlock first.
 */
static int flash_write_halfword(uint32_t addr, uint16_t data)
{
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint64_t)data)
        != HAL_OK) {
        return 0;
    }
    /* Verify */
    if (*(__IO uint16_t *)addr != data) return 0;
    return 1;
}

/**
 * Write a buffer of uint8_t to flash.
 * Handles unaligned start: pads odd-byte start with 0xFF in the same
 * half-word, and handles an odd final byte.
 */
static int flash_write_buf(uint32_t addr, const uint8_t *data, uint16_t len)
{
    /* If addr is odd, read-modify-write the first halfword */
    if (addr & 1) {
        uint16_t hw = 0xFF00 | data[0];
        if (!flash_write_halfword(addr & ~1U, hw)) return 0;
        addr += 1;
        data += 1;
        len  -= 1;
    }

    while (len >= 2) {
        uint16_t hw = ((uint16_t)data[0])
                    | ((uint16_t)data[1] << 8);
        if (!flash_write_halfword(addr, hw)) return 0;
        addr += 2;
        data += 2;
        len  -= 2;
    }

    /* Last odd byte */
    if (len == 1) {
        uint16_t hw = (uint16_t)data[0] | 0xFF00U;
        if (!flash_write_halfword(addr, hw)) return 0;
    }

    return 1;
}

/* ================================================================
 *  CRC-16 for stored config integrity check
 * ================================================================ */

static uint16_t calc_crc(const uint8_t *data, uint16_t len)
{
    return crc16_modbus(data, len);
}

/* ================================================================
 *  Load from flash
 * ================================================================ */

int flash_load_all(void)
{
    const uint8_t *src = (const uint8_t *)FLASH_STORAGE_ADDR;

    /* 1. Check magic */
    uint32_t magic;
    memcpy(&magic, src + FLASH_OFF_MAGIC, 4);
    if (magic != FLASH_MAGIC) {
        return 0;   /* never written — use RAM defaults */
    }

    /* 2. CRC check (everything before CRC field) */
    uint16_t crc_stored;
    memcpy(&crc_stored, src + FLASH_OFF_CRC, 2);
    if (crc_stored != calc_crc(src, FLASH_OFF_CRC)) {
        return 0;   /* corrupt */
    }

    /* 3. Load config */
    memcpy(g_config.mac,     src + FLASH_OFF_MAC,     6);
    memcpy(g_config.ip,      src + FLASH_OFF_IP,      4);
    memcpy(g_config.subnet,  src + FLASH_OFF_SUBNET,  4);
    memcpy(g_config.gateway, src + FLASH_OFF_GATEWAY, 4);
    g_config.fw_version = *(const uint16_t *)(src + FLASH_OFF_FW_VERSION);

    /* 4. Load matrix (144 bytes float32) */
    memcpy(&g_matrix, src + FLASH_OFF_MATRIX, sizeof(decouple_matrix_t));

    /* 5. Load zero offsets (24 bytes float32) */
    memcpy((void *)g_sensor.force_zero, src + FLASH_OFF_ZERO,
           6 * sizeof(float));

    return 1;
}

/* ================================================================
 *  Save to flash  (erase-then-write-all pattern)
 * ================================================================ */

/* Temporary buffer for assembling a full page image */
static uint8_t page_buf[256];

static int flash_save_all(void)
{
    uint8_t *buf = page_buf;
    memset(buf, 0xFF, sizeof(page_buf));

    /* Copy current RAM values into the buffer */
    memcpy(buf + FLASH_OFF_MAC,       g_config.mac,      6);
    memcpy(buf + FLASH_OFF_IP,        g_config.ip,       4);
    memcpy(buf + FLASH_OFF_SUBNET,    g_config.subnet,   4);
    memcpy(buf + FLASH_OFF_GATEWAY,   g_config.gateway,  4);
    memcpy(buf + FLASH_OFF_MATRIX,    &g_matrix,         144);
    memcpy(buf + FLASH_OFF_ZERO,      (const void *)g_sensor.force_zero, 24);
    memcpy(buf + FLASH_OFF_FW_VERSION,&g_config.fw_version, 2);

    /* Magic */
    uint32_t magic = FLASH_MAGIC;
    memcpy(buf + FLASH_OFF_MAGIC, &magic, 4);

    /* CRC */
    uint16_t crc = calc_crc(buf, FLASH_OFF_CRC);
    memcpy(buf + FLASH_OFF_CRC, &crc, 2);

    /* Flash program */
    flash_unlock();
    flash_erase_storage();

    int ok = flash_write_buf(FLASH_STORAGE_ADDR, buf, FLASH_OFF_CRC + 2);

    flash_lock();
    return ok;
}

/* ---- Individual save functions (convenience wrappers) ---- */

int flash_save_config(void)
{
    HAL_FLASH_Unlock();
    flash_erase_storage();

    uint8_t buf[256];
    memset(buf, 0xFF, sizeof(buf));

    /* Read back existing matrix and zero from flash before erasing...
       Actually we need to preserve them.  Since erase destroys everything,
       the simplest approach is to rebuild the full page from RAM.
       All data lives in RAM anyway. */
    memcpy(buf + FLASH_OFF_MAC,       g_config.mac,      6);
    memcpy(buf + FLASH_OFF_IP,        g_config.ip,       4);
    memcpy(buf + FLASH_OFF_SUBNET,    g_config.subnet,   4);
    memcpy(buf + FLASH_OFF_GATEWAY,   g_config.gateway,  4);
    memcpy(buf + FLASH_OFF_MATRIX,    &g_matrix,         144);
    memcpy(buf + FLASH_OFF_ZERO,
           (const void *)g_sensor.force_zero, 24);
    memcpy(buf + FLASH_OFF_FW_VERSION, &g_config.fw_version, 2);

    uint32_t magic = FLASH_MAGIC;
    memcpy(buf + FLASH_OFF_MAGIC, &magic, 4);
    uint16_t crc = calc_crc(buf, FLASH_OFF_CRC);
    memcpy(buf + FLASH_OFF_CRC, &crc, 2);

    int ok = flash_write_buf(FLASH_STORAGE_ADDR, buf, FLASH_OFF_CRC + 2);
    HAL_FLASH_Lock();
    return ok;
}

int flash_save_matrix(void)
{
    return flash_save_all();
}

int flash_save_zero(void)
{
    return flash_save_all();
}
