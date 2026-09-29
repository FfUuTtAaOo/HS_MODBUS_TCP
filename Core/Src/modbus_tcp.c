#include "modbus_tcp.h"
#include "mb_register.h"
#include <string.h>

/* MBAP header offsets */
#define MBAP_TID     0   /* Transaction ID, 2 bytes */
#define MBAP_PID     2   /* Protocol ID,   2 bytes (=0) */
#define MBAP_LEN     4   /* Length,        2 bytes */
#define MBAP_UID     6   /* Unit ID,       1 byte  */
#define MBAP_HDR_LEN 7

/* ---- Internal ---- */

static void build_echo(uint8_t *buf, const uint8_t *rx, uint8_t fc,
                       const uint8_t *data, uint16_t data_len)
{
    /* Copy MBAP header (first 6 bytes) verbatim, then fill len+uid+FC */
    memcpy(buf, rx, 6);
    uint16_t pdu_len = (uint16_t)(2 + data_len);        /* UID + FC + data */
    buf[MBAP_LEN]     = (uint8_t)(pdu_len >> 8);
    buf[MBAP_LEN + 1] = (uint8_t)(pdu_len);
    buf[MBAP_UID]     = rx[MBAP_UID];
    buf[MBAP_HDR_LEN] = fc;
    if (data_len && data) memcpy(buf + MBAP_HDR_LEN + 1, data, data_len);
}

static void build_exception(uint8_t *buf, const uint8_t *rx, uint8_t exc)
{
    memcpy(buf, rx, 6);
    buf[MBAP_LEN]     = 0x00;
    buf[MBAP_LEN + 1] = 0x03;               /* UID + FC + exc = 3 */
    buf[MBAP_UID]     = rx[MBAP_UID];
    buf[MBAP_HDR_LEN] = (uint8_t)(rx[MBAP_HDR_LEN] | 0x80);
    buf[MBAP_HDR_LEN + 1] = exc;
}

/* ================================================================
 *  modbus_tcp_process
 * ================================================================ */
int modbus_tcp_process(uint8_t sock, const uint8_t *rx_buf, uint16_t rx_len,
                       mb_tcp_send_fn send_fn, void *ctx)
{
    /* ---- 1. Validate MBAP header ---- */
    if (rx_len < MBAP_HDR_LEN + 1) return -1;   /* no FC byte */
    if ((((uint16_t)rx_buf[MBAP_PID] << 8) | rx_buf[MBAP_PID + 1]) != 0)
        return -1;

    uint16_t pdu_len = ((uint16_t)rx_buf[MBAP_LEN] << 8) | rx_buf[MBAP_LEN + 1];
    if (rx_len < (uint16_t)(MBAP_HDR_LEN + pdu_len - 1))
        return -1;                             /* incomplete PDU             */

    uint8_t  fc      = rx_buf[MBAP_HDR_LEN];
    uint8_t  tx[MB_TCP_TX_BUF_SIZE];
    uint8_t  exc     = MB_EX_NONE;
    uint16_t resp_len = 0;

    /* ---- 2. Dispatch ---- */
    switch (fc) {

    /* Read Holding Registers (0x03) */
    case MB_FC_READ_HOLDING: {
        if (pdu_len < 5) { exc = MB_EX_ILLEGAL_DATA; break; }
        uint16_t addr  = ((uint16_t)rx_buf[MBAP_HDR_LEN + 1] << 8)
                       |  rx_buf[MBAP_HDR_LEN + 2];
        uint16_t count = ((uint16_t)rx_buf[MBAP_HDR_LEN + 3] << 8)
                       |  rx_buf[MBAP_HDR_LEN + 4];
        uint8_t  data[250];
        uint16_t bytes = mb_reg_read(addr, count, data, &exc);
        if (exc != MB_EX_NONE) break;

        build_echo(tx, rx_buf, fc, NULL, (uint16_t)(1 + bytes));
        tx[MBAP_HDR_LEN + 1] = (uint8_t)bytes;
        memcpy(tx + MBAP_HDR_LEN + 2, data, bytes);
        resp_len = (uint16_t)(MBAP_HDR_LEN + 2 + bytes);
        break;
    }

    /* Write Single Register (0x06) */
    case MB_FC_WRITE_SINGLE: {
        if (pdu_len < 5) { exc = MB_EX_ILLEGAL_DATA; break; }
        uint16_t addr  = ((uint16_t)rx_buf[MBAP_HDR_LEN + 1] << 8)
                       |  rx_buf[MBAP_HDR_LEN + 2];
        uint16_t value = ((uint16_t)rx_buf[MBAP_HDR_LEN + 3] << 8)
                       |  rx_buf[MBAP_HDR_LEN + 4];
        (void)mb_reg_write_single(addr, value, &exc);
        if (exc != MB_EX_NONE) break;

        /* Echo entire request */
        build_echo(tx, rx_buf, fc,
                   rx_buf + MBAP_HDR_LEN + 1, 4);
        resp_len = (uint16_t)(6 + pdu_len);
        break;
    }

    /* Write Multiple Registers (0x10) */
    case MB_FC_WRITE_MULTI: {
        if (pdu_len < 7) { exc = MB_EX_ILLEGAL_DATA; break; }
        uint16_t addr     = ((uint16_t)rx_buf[MBAP_HDR_LEN + 1] << 8)
                          |  rx_buf[MBAP_HDR_LEN + 2];
        uint16_t count    = ((uint16_t)rx_buf[MBAP_HDR_LEN + 3] << 8)
                          |  rx_buf[MBAP_HDR_LEN + 4];
        uint8_t  byte_cnt = rx_buf[MBAP_HDR_LEN + 5];
        if (byte_cnt != count * 2) { exc = MB_EX_ILLEGAL_DATA; break; }
        (void)mb_reg_write_multi(addr, count,
                                 rx_buf + MBAP_HDR_LEN + 6, &exc);
        if (exc != MB_EX_NONE) break;

        /* Response: echo MBAP + FC + addr + count */
        uint8_t echo_data[4];
        echo_data[0] = (uint8_t)(addr >> 8);
        echo_data[1] = (uint8_t)(addr);
        echo_data[2] = (uint8_t)(count >> 8);
        echo_data[3] = (uint8_t)(count);
        build_echo(tx, rx_buf, fc, echo_data, 4);
        resp_len = MBAP_HDR_LEN + 1 + 4;
        break;
    }

    default:
        exc = MB_EX_ILLEGAL_FC;
        break;
    }

    /* ---- 3. Build exception if failed ---- */
    if (exc != MB_EX_NONE) {
        build_exception(tx, rx_buf, exc);
        resp_len = MBAP_HDR_LEN + 2;
    }

    /* ---- 4. Send response ---- */
    if (send_fn) send_fn(sock, tx, resp_len, ctx);
    return (exc != MB_EX_NONE) ? (int)exc : 0;
}
