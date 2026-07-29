#include "mb_register.h"
#include "calibration.h"
#include "flash_storage.h"
#include "self_test.h"
#include <string.h>

extern uint8_t output_interface;

/* ================================================================
 *  Internal helpers
 * ================================================================ */

static float buf_to_float(const uint8_t *src)
{
    uint32_t u = ((uint32_t)src[0])
               | ((uint32_t)src[1] << 8)
               | ((uint32_t)src[2] << 16)
               | ((uint32_t)src[3] << 24);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

/* Block descriptor for valid_range check */
typedef struct {
    uint16_t start;
    uint16_t end;   /* inclusive */
} reg_block_t;

/* Validate register range: must lie entirely within one defined block */
static int valid_range(uint16_t addr, uint16_t count, uint8_t *exc)
{
    if (count == 0 || count > 125) { *exc = MB_EX_ILLEGAL_DATA; return 0; }

    static const reg_block_t blocks[] = {
        { MB_BLK_TRIG_START,  MB_BLK_TRIG_END   },   /* STOP/START/SINGLE    */
        { MB_BLK_INFO_START,  MB_BLK_INFO_END    },   /* SN+FW+Status+CommErr */
        { MB_BLK_CTRL_START,  MB_BLK_CTRL_END    },   /* ZERO/UNZERO+FORMAT+FORCE */
        { MB_BLK_ZERO_START,  MB_BLK_ZERO_END    },   /* zero offsets         */
        { MB_BLK_OVL_START,   MB_BLK_OVL_END     },   /* overload thresholds  */
        { MB_BLK_RANGE_START, MB_BLK_RANGE_END   },   /* threshold ranges     */
        { MB_BLK_NET_START,   MB_BLK_NET_END     },   /* network config       */
        { MB_BLK_EXT_START,   MB_BLK_EXT_END     },   /* freq_mode            */
        { MB_BLK_ST_START,    MB_BLK_ST_END      },   /* self-test            */
    };

    uint16_t end_addr = addr + count - 1;
    for (size_t i = 0; i < sizeof(blocks) / sizeof(blocks[0]); i++) {
        if (addr >= blocks[i].start && end_addr <= blocks[i].end) {
            return 1;
        }
    }

    *exc = MB_EX_ILLEGAL_ADDR;
    return 0;
}

/* ================================================================
 *  mb_reg_read  —  Read Holding Registers (FC 03)
 * ================================================================ */
uint16_t mb_reg_read(uint16_t addr, uint16_t count,
                     uint8_t *buf, uint8_t *exc)
{
    *exc = MB_EX_NONE;
    if (!valid_range(addr, count, exc)) return 0;

    for (uint16_t i = 0, off = 0; i < count; i++) {
        uint16_t val = 0;
        uint16_t a   = addr + i;

        switch (a) {
        /* ---- Write-trigger registers — always return 0 ---- */
        case MB_REG_STOP:
        case MB_REG_START:
        case MB_REG_SINGLE:
        case MB_REG_ZERO_TRIG:
        case MB_REG_UNZERO_TRIG:
            val = 0; break;

        /* ---- SN: 8 words of ASCII (16 chars) ---- */
        case MB_REG_SN_BASE ... MB_REG_SN_END:
            val = ((uint16_t)(uint8_t)g_config.sn[(a - MB_REG_SN_BASE) * 2])
                | ((uint16_t)(uint8_t)g_config.sn[(a - MB_REG_SN_BASE) * 2 + 1] << 8);
            break;

        /* ---- FW Version: 2 registers (major | minor) from BCD ---- */
        case MB_REG_FW_VERSION:     /* major */
            val = (g_config.fw_version >> 8) & 0xFF; break;
        case MB_REG_FW_VERSION + 1: /* minor */
            val = g_config.fw_version & 0xFF; break;

        /* ---- Status / Comm Error / Data Format ---- */
        case MB_REG_STATUS:
            val = g_sys.status_flags; break;
        case MB_REG_COMM_ERR:
            val = g_sys.comm_error_cnt; break;
        case MB_REG_DATA_FORMAT:
            val = g_sys.data_format; break;

        /* ---- Force / Torque (float32 LE, 2 words each) ---- */
        case MB_REG_FORCE_FX ... MB_REG_TORQUE_MZ: {
            int ch = (int)(a - MB_REG_FORCE_FX) / 2;
            int lo = ((a - MB_REG_FORCE_FX) % 2) == 0;
            uint32_t u;
            float __tmp = g_sensor.force[ch];
            memcpy(&u, &__tmp, 4);
            val = lo ? (uint16_t)u : (uint16_t)(u >> 16);
            break;
        }

        /* ---- Zero Offsets (float32 LE, 2 words each) ---- */
        case MB_REG_ZERO_FX ... MB_REG_ZERO_MZ + 1: {
            int ch = (int)(a - MB_REG_ZERO_FX) / 2;
            int lo = ((a - MB_REG_ZERO_FX) % 2) == 0;
            uint32_t u;
            float __tmp = g_sensor.force_zero[ch];
            memcpy(&u, &__tmp, 4);
            val = lo ? (uint16_t)u : (uint16_t)(u >> 16);
            break;
        }

        /* ---- Overload Thresholds (float32 LE, 2 words each) ---- */
        case MB_REG_OVL_FX ... MB_REG_OVL_MZ + 1: {
            int ch = (int)(a - MB_REG_OVL_FX) / 2;
            int lo = ((a - MB_REG_OVL_FX) % 2) == 0;
            uint32_t u;
            memcpy(&u, &g_threshold.overload[ch], 4);
            val = lo ? (uint16_t)u : (uint16_t)(u >> 16);
            break;
        }

        /* ---- Threshold Range (float32 LE, 4 words per axis: min lo/hi, max lo/hi) ---- */
        case MB_REG_RANGE_FX_MIN ... MB_REG_RANGE_MZ_MAX: {
            int ch  = (int)(a - MB_REG_RANGE_FX_MIN) / 4;
            int sub = (int)(a - MB_REG_RANGE_FX_MIN) % 4;  /* 0/1=min, 2/3=max */
            float f = (sub < 2) ? g_threshold.range_min[ch] : g_threshold.range_max[ch];
            uint32_t u;
            memcpy(&u, &f, 4);
            val = (sub % 2 == 0) ? (uint16_t)u : (uint16_t)(u >> 16);
            break;
        }

        /* ---- MAC: 6 words — first 3 hold real data, rest are zero ---- */
        case MB_REG_MAC:
            val = ((uint16_t)g_config.mac[0] << 8) | g_config.mac[1]; break;
        case MB_REG_MAC + 1:
            val = ((uint16_t)g_config.mac[2] << 8) | g_config.mac[3]; break;
        case MB_REG_MAC + 2:
            val = ((uint16_t)g_config.mac[4] << 8) | g_config.mac[5]; break;
        case MB_REG_MAC + 3 ... MB_REG_MAC + 5:
            val = 0; break;

        /* ---- IP / Subnet / Gateway: 2 words each ---- */
        case MB_REG_IP ... MB_REG_IP + 1:
            val = ((uint16_t)g_config.ip[(a - MB_REG_IP) * 2] << 8)
                |  g_config.ip[(a - MB_REG_IP) * 2 + 1]; break;
        case MB_REG_SUBNET ... MB_REG_SUBNET + 1:
            val = ((uint16_t)g_config.subnet[(a - MB_REG_SUBNET) * 2] << 8)
                |  g_config.subnet[(a - MB_REG_SUBNET) * 2 + 1]; break;
        case MB_REG_GATEWAY ... MB_REG_GATEWAY + 1:
            val = ((uint16_t)g_config.gateway[(a - MB_REG_GATEWAY) * 2] << 8)
                |  g_config.gateway[(a - MB_REG_GATEWAY) * 2 + 1]; break;

        /* ---- Extension: freq_mode ---- */
        case MB_REG_FREQ_MODE:
            val = g_sys.freq_mode; break;

        /* ---- Self-test ---- */
        case MB_REG_ST_ERROR:
            val = (uint16_t)(g_self_test.error_flags & 0xFFFF); break;
        case MB_REG_ST_ERROR_HI:
            val = (uint16_t)(g_self_test.error_flags >> 16); break;
        case MB_REG_ST_ADC_ID:
            val = ((uint16_t)g_self_test.adc_id << 8) | g_self_test.adc_error; break;
        case MB_REG_ST_W5500:
            val = ((uint16_t)g_self_test.w5500_version << 8)
                | ((uint16_t)(g_self_test.w5500_phy_link ? 0x80 : 0)
                   | (g_self_test.w5500_sockets_ok & 0x0F)); break;
        case MB_REG_ST_RS485:
            val = ((uint16_t)(g_self_test.rs485_uart_ok ? 0x80 : 0)
                 | (uint16_t)(g_self_test.rs485_dma_ok ? 0x40 : 0)); break;
        case MB_REG_ST_FLASH:
            val = ((uint16_t)(g_self_test.flash_valid    ? 0x80 : 0)
                 | (uint16_t)(g_self_test.flash_matrix_ok ? 0x40 : 0)
                 | (uint16_t)(g_self_test.flash_zero_ok   ? 0x20 : 0)
                 | (uint16_t)(g_self_test.flash_config_ok ? 0x10 : 0)); break;

        default:
            *exc = MB_EX_ILLEGAL_ADDR; return 0;
        }

        buf[off++] = (uint8_t)(val >> 8);
        buf[off++] = (uint8_t)(val);
    }
    return count * 2U;
}

/* ================================================================
 *  mb_reg_write_single  —  Write Single Register (FC 06)
 * ================================================================ */
uint16_t mb_reg_write_single(uint16_t addr, uint16_t value, uint8_t *exc)
{
    *exc = MB_EX_NONE;

    switch (addr) {
    /* ---- Write-trigger commands (value is ignored per protocol) ---- */
    case MB_REG_STOP:
        g_sys.send_mode = 0;
        output_interface = 0;
        break;
    case MB_REG_START:
        g_sys.send_mode = 1;
        output_interface = 2;
        break;
    case MB_REG_SINGLE:
        g_sys.send_mode = 2;
        break;
    case MB_REG_ZERO_TRIG:
        calib_zero_start();
        g_sys.zero_calib_busy = 1;
        break;
    case MB_REG_UNZERO_TRIG:
        calib_zero_cancel();
        flash_save_zero();
        break;

    /* ---- R/W registers ---- */
    case MB_REG_DATA_FORMAT:
        if (value > 2) { *exc = MB_EX_ILLEGAL_DATA; return 0; }
        g_sys.data_format = (uint8_t)value;
        break;
    case MB_REG_STATUS:
        g_sys.status_flags = value;
        break;

    /* ---- float32 Zero Offsets (single-register write via FC 06 not allowed;
            use FC 10 to write full float32 pairs) ---- */
    /* ---- Network config: single-reg write also not allowed — use FC 10 ---- */

    /* ---- Extension ---- */
    case MB_REG_FREQ_MODE:
        if (value > 1) { *exc = MB_EX_ILLEGAL_DATA; return 0; }
        g_sys.freq_mode = (uint8_t)value;
        break;

    default:
        *exc = MB_EX_ILLEGAL_ADDR; return 0;
    }

    return 2U;   /* echo back 1 register = 2 bytes */
}

/* ================================================================
 *  mb_reg_write_multi — Write Multiple Registers (FC 10)
 * ================================================================ */
uint16_t mb_reg_write_multi(uint16_t addr, uint16_t count,
                            const uint8_t *data, uint8_t *exc)
{
    *exc = MB_EX_NONE;
    if (!valid_range(addr, count, exc)) return 0;

    /* Network config block: MAC(6) + IP(2) + Subnet(2) + Gateway(2) = 12 words */
    if (addr >= MB_BLK_NET_START && addr <= MB_BLK_NET_END) {
        uint16_t end = addr + count;
        for (uint16_t off = 0; addr < end; addr++, off += 2) {
            uint16_t val = ((uint16_t)data[off] << 8) | data[off + 1];
            if (addr >= MB_REG_MAC && addr < MB_REG_MAC + 3) {
                g_config.mac[(addr - MB_REG_MAC) * 2]     = (uint8_t)(val >> 8);
                g_config.mac[(addr - MB_REG_MAC) * 2 + 1] = (uint8_t)(val);
            } else if (addr >= MB_REG_MAC + 3 && addr < MB_REG_IP) {
                /* padding registers within MAC range — silently ignore */
            } else if (addr >= MB_REG_IP && addr < MB_REG_IP + 2) {
                g_config.ip[(addr - MB_REG_IP) * 2]     = (uint8_t)(val >> 8);
                g_config.ip[(addr - MB_REG_IP) * 2 + 1] = (uint8_t)(val);
            } else if (addr >= MB_REG_SUBNET && addr < MB_REG_SUBNET + 2) {
                g_config.subnet[(addr - MB_REG_SUBNET) * 2]     = (uint8_t)(val >> 8);
                g_config.subnet[(addr - MB_REG_SUBNET) * 2 + 1] = (uint8_t)(val);
            } else if (addr >= MB_REG_GATEWAY && addr < MB_REG_GATEWAY + 2) {
                g_config.gateway[(addr - MB_REG_GATEWAY) * 2]     = (uint8_t)(val >> 8);
                g_config.gateway[(addr - MB_REG_GATEWAY) * 2 + 1] = (uint8_t)(val);
            }
        }
        return count * 2U;
    }

    /* Zero offset block: 6 × float32 = 12 words */
    if (addr >= MB_BLK_ZERO_START && addr <= MB_BLK_ZERO_END) {
        if (count % 2 != 0) { *exc = MB_EX_ILLEGAL_DATA; return 0; }
        uint16_t end = addr + count;
        for (uint16_t data_off = 0; addr < end; addr += 2, data_off += 4) {
            int ch = (int)(addr - MB_REG_ZERO_FX) / 2;
            if (ch < 0 || ch >= 6) { *exc = MB_EX_ILLEGAL_ADDR; return 0; }
            g_sensor.force_zero[ch] = buf_to_float(data + data_off);
        }
        return count * 2U;
    }

    /* Overload threshold block: 6 × float32 = 12 words */
    if (addr >= MB_BLK_OVL_START && addr <= MB_BLK_OVL_END) {
        if (count % 2 != 0) { *exc = MB_EX_ILLEGAL_DATA; return 0; }
        uint16_t end = addr + count;
        for (uint16_t data_off = 0; addr < end; addr += 2, data_off += 4) {
            int ch = (int)(addr - MB_REG_OVL_FX) / 2;
            if (ch < 0 || ch >= 6) { *exc = MB_EX_ILLEGAL_ADDR; return 0; }
            g_threshold.overload[ch] = buf_to_float(data + data_off);
        }
        return count * 2U;
    }

    /* Threshold range block: 12 × float32 (min+max per axis) = 24 words */
    if (addr >= MB_BLK_RANGE_START && addr <= MB_BLK_RANGE_END) {
        if (count % 2 != 0) { *exc = MB_EX_ILLEGAL_DATA; return 0; }
        uint16_t end = addr + count;
        for (uint16_t data_off = 0; addr < end; addr += 2, data_off += 4) {
            int off    = (int)(addr - MB_REG_RANGE_FX_MIN);
            int ch     = off / 4;
            int is_min = (off % 4) < 2;
            if (ch < 0 || ch >= 6) { *exc = MB_EX_ILLEGAL_ADDR; return 0; }
            if (is_min)
                g_threshold.range_min[ch] = buf_to_float(data + data_off);
            else
                g_threshold.range_max[ch] = buf_to_float(data + data_off);
        }
        return count * 2U;
    }

    /* For single-register addresses, fallback to single write */
    if (count == 1) {
        uint16_t val = ((uint16_t)data[0] << 8) | data[1];
        return mb_reg_write_single(addr, val, exc);
    }

    *exc = MB_EX_ILLEGAL_ADDR;
    return 0;
}
