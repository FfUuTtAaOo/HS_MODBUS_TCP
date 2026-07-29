#include "rs485_cmd.h"
#include "rs485.h"
#include "sensor_types.h"
#include "calibration.h"
#include "flash_storage.h"
#include <string.h>

extern uint8_t output_interface;

/* ================================================================
 *  Internal helpers
 * ================================================================ */

static void float_to_le(uint8_t *dst, float f)
{
    uint32_t u; memcpy(&u, &f, 4);
    dst[0] = (uint8_t)(u);
    dst[1] = (uint8_t)(u >> 8);
    dst[2] = (uint8_t)(u >> 16);
    dst[3] = (uint8_t)(u >> 24);
}

static float le_to_float(const uint8_t *src)
{
    uint32_t u = ((uint32_t)src[3])
               | ((uint32_t)src[2] << 8)
               | ((uint32_t)src[1] << 16)
               | ((uint32_t)src[0] << 24);
    float f; memcpy(&f, &u, 4); return f;
}

/* Send a response frame: AA 55 [cmd|0x80] [data...] 0D 0A */
static void respond(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    rs485_send_frame((uint8_t)(cmd | RS485_RESP_FLAG), data, len);
}

/* Send a single-byte ACK response */
static void ack(uint8_t cmd) { respond(cmd, NULL, 0); }

/* ================================================================
 *  rs485_cmd_dispatch
 * ================================================================ */
void rs485_cmd_dispatch(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[150];   /* enough for 144-byte matrix ack */

    switch (cmd) {

    /* ---- 0x01  Stop continuous send ---- */
    case RS485_CMD_STOP:
        g_sys.send_mode = 0;
        output_interface = 0;
        ack(cmd);
        break;

    /* ---- 0x02  Start continuous send ---- */
    case RS485_CMD_START:
        g_sys.send_mode = 1;
        output_interface = 1;
        ack(cmd);
        break;

    /* ---- 0x03  Single-shot request ---- */
    case RS485_CMD_SINGLE: {
        /* Respond with one data frame: AA 55 00 [SEQ4B] [6×float32] 0D 0A */
        uint32_t seq = g_sys.frame_seq++;
        buf[0] = RS485_CMD_CONT_DATA;                  /* CMD */
        buf[1] = (uint8_t)(seq);
        buf[2] = (uint8_t)(seq >> 8);
        buf[3] = (uint8_t)(seq >> 16);
        buf[4] = (uint8_t)(seq >> 24);
        for (int i = 0; i < 6; i++) {
            float_to_le(buf + 5 + i * 4, g_sensor.force[i]);
        }
        rs485_send_raw(buf, 5 + 24);
        break;
    }

    /* ---- 0x04  Set frequency ---- */
    case RS485_CMD_SET_FREQ:
        if (len < 1) { g_sys.comm_error_cnt++; break; }
        g_sys.freq_mode = (payload[0] == 0) ? 0U : 1U;
        ack(cmd);
        break;

    /* ---- 0x05  Query serial number ---- */
    case RS485_CMD_QUERY_SN:
        respond(cmd, (const uint8_t *)g_config.sn, (uint16_t)strlen(g_config.sn));
        break;

    /* ---- 0x06  Query firmware version ---- */
    case RS485_CMD_QUERY_VER:
        buf[0] = (uint8_t)(g_config.fw_version >> 8);
        buf[1] = (uint8_t)(g_config.fw_version);
        respond(cmd, buf, 2);
        break;

    /* ---- 0x07  Query system status ---- */
    case RS485_CMD_QUERY_STATUS:
        buf[0] = g_sys.send_mode;
        buf[1] = g_sys.data_format;
        buf[2] = g_sys.freq_mode;
        buf[3] = g_sys.debug_mode;
        buf[4] = (uint8_t)(g_sys.status_flags);
        buf[5] = (uint8_t)(g_sys.status_flags >> 8);
        buf[6] = (uint8_t)(g_sys.comm_error_cnt);
        buf[7] = (uint8_t)(g_sys.comm_error_cnt >> 8);
        respond(cmd, buf, 8);
        break;

    /* ---- 0x30  Zero calibration (20-point force average) ---- */
    case RS485_CMD_ZERO:
        calib_zero_start();
        g_sys.zero_calib_busy = 1;
        ack(cmd);
        break;

    case RS485_CMD_HEX_SEND:
        g_sys.debug_mode = 0;
        ack(cmd);
        break;

    case RS485_CMD_ASC_SEND:
        g_sys.debug_mode = 1;
        ack(cmd);
        break;

    /* ---- 0x36  Cancel zero ---- */
    case RS485_CMD_UNZERO:
        calib_zero_cancel();
        flash_save_zero();
        ack(cmd);
        break;


    /* ---- 0x34  Set data format ---- */
    case RS485_CMD_SET_FORMAT:
        // if (len < 1) { g_sys.comm_error_cnt++; break; }
        // if (payload[0] <= 2) g_sys.data_format = payload[0];
        // else                  g_sys.comm_error_cnt++;
        // ack(cmd);
        g_sys.data_format = 0;
        ack(cmd);
        break;

    /* ---- 0x33  Set data format ---- */
    case RS485_CMD_SET_FORMAT_KG:
        g_sys.data_format = 1;
        ack(cmd);
        break;

    /* ---- 0x35  Set data format ---- */
    case RS485_CMD_SET_FORMAT_N:
        g_sys.data_format = 2;
        ack(cmd);
        break;

    /* ---- 0x0B  Set IP ---- */
    case RS485_CMD_SET_IP:
        if (len < 4) { g_sys.comm_error_cnt++; break; }
        memcpy(g_config.ip, payload, 4);
        flash_save_config();
        ack(cmd);
        break;

    /* ---- 0x0C  Set MAC ---- */
    case RS485_CMD_SET_MAC:
        if (len < 6) { g_sys.comm_error_cnt++; break; }
        memcpy(g_config.mac, payload, 6);
        flash_save_config();
        ack(cmd);
        break;

    /* ---- 0x0D  Set subnet ---- */
    case RS485_CMD_SET_SUBNET:
        if (len < 4) { g_sys.comm_error_cnt++; break; }
        memcpy(g_config.subnet, payload, 4);
        flash_save_config();
        ack(cmd);
        break;

    /* ---- 0x0E  Set gateway ---- */
    case RS485_CMD_SET_GW:
        if (len < 4) { g_sys.comm_error_cnt++; break; }
        memcpy(g_config.gateway, payload, 4);
        flash_save_config();
        ack(cmd);
        break;

    /* ---- 0x0F  Query network config ---- */
    case RS485_CMD_QUERY_NET: {
        uint16_t pos = 0;
        memcpy(buf + pos, g_config.mac, 6);     pos += 6;
        memcpy(buf + pos, g_config.ip, 4);       pos += 4;
        memcpy(buf + pos, g_config.subnet, 4);   pos += 4;
        memcpy(buf + pos, g_config.gateway, 4);  pos += 4;
        respond(cmd, buf, pos);
        break;
    }

    /* ---- 0x20  Upload decoupling matrix (6×6 × 4 = 144 bytes) ---- */
    case RS485_CMD_SET_MATRIX:
        // if (len < 144) { g_sys.comm_error_cnt++; break; }
        // for (int i = 0; i < 6; i++) {
        //     for (int j = 0; j < 6; j++) {
        //         g_matrix.m[i][j] = le_to_float(payload + (i * 6 + j) * 4);
        //     }
        // }
        // flash_save_matrix();
        // ack(cmd);

        if (len < 25) { g_sys.comm_error_cnt++; break; }
        for (int i = 0; i < 6; i++) {
            g_matrix.m[payload[0]][i] = le_to_float(payload + 1 + i * 4);
        }
        flash_save_matrix();
        ack(cmd | payload[0]);
        break;

    /* ---- 0x35  Debug mode toggle ---- */
    // case RS485_CMD_DEBUG_MODE:
    //     g_sys.debug_mode = (uint8_t)(g_sys.debug_mode ? 0U : 1U);
    //     buf[0] = g_sys.debug_mode;
        // respond(cmd, buf, 1);
    //     break;

    default:
        /* Unknown command — silently ignored */
        g_sys.comm_error_cnt++;
        break;
    }
}
