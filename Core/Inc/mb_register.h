#ifndef __MB_REGISTER_H__
#define __MB_REGISTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sensor_types.h"

/* ================================================================
 *  Modbus Register Address Map
 *
 *  Addresses aligned with modbus-tcp-protocol-v2.0 (sent to customer).
 *  All registers are Holding Registers (function codes 03/06/10).
 *  float32 values occupy 2 consecutive registers (little-endian).
 * ================================================================ */

/* ---- Write-Trigger Registers (W only via FC 06, auto-reset to 0) -- */
#define MB_REG_STOP          0x0001  /* Write any → send_mode=0           */
#define MB_REG_START         0x0002  /* Write any → send_mode=1           */
#define MB_REG_SINGLE        0x0003  /* Write any → send_mode=2           */

/* ---- System Info (read-only) ------------------------------------ */
#define MB_REG_SN_BASE       0x0005  /* SN string, 8 words (16 ASCII)     */
#define MB_REG_SN_END        0x000C
#define MB_REG_FW_VERSION    0x000D  /* FW version, 2 words (major,minor) */
#define MB_REG_STATUS        0x000F  /* BIT0=auto-start, BIT1=active      */
#define MB_REG_COMM_ERR      0x0010  /* Comm error count (read-only)      */

/* ---- Control (write-trigger / read-write) ----------------------- */
#define MB_REG_ZERO_TRIG     0x0030  /* Write any → zero calibration      */
#define MB_REG_UNZERO_TRIG   0x0031  /* Write any → cancel zero           */
#define MB_REG_DATA_FORMAT   0x0032  /* 0=mV, 1=kg, 2=N                  */

/* ---- Force / Torque (read-only, float32 LE ×2 words each) ------- */
#define MB_REG_FORCE_FX      0x0033
#define MB_REG_FORCE_FY      0x0035
#define MB_REG_FORCE_FZ      0x0037
#define MB_REG_TORQUE_MX     0x0039
#define MB_REG_TORQUE_MY     0x003B
#define MB_REG_TORQUE_MZ     0x003D

/* ---- Zero Offsets (read/write, float32 LE ×2 words each) -------- */
#define MB_REG_ZERO_FX       0x0050
#define MB_REG_ZERO_FY       0x0052
#define MB_REG_ZERO_FZ       0x0054
#define MB_REG_ZERO_MX       0x0056
#define MB_REG_ZERO_MY       0x0058
#define MB_REG_ZERO_MZ       0x005A

/* ---- Overload Thresholds (R/W, float32 LE ×2 words each) -------- */
#define MB_REG_OVL_FX        0x0070
#define MB_REG_OVL_FY        0x0072
#define MB_REG_OVL_FZ        0x0074
#define MB_REG_OVL_MX        0x0076
#define MB_REG_OVL_MY        0x0078
#define MB_REG_OVL_MZ        0x007A

/* ---- Threshold Range (R/W, float32 LE, min+max per axis) -------- */
#define MB_REG_RANGE_FX_MIN  0x007C
#define MB_REG_RANGE_FX_MAX  0x007E
#define MB_REG_RANGE_FY_MIN  0x0080
#define MB_REG_RANGE_FY_MAX  0x0082
#define MB_REG_RANGE_FZ_MIN  0x0084
#define MB_REG_RANGE_FZ_MAX  0x0086
#define MB_REG_RANGE_MX_MIN  0x0088
#define MB_REG_RANGE_MX_MAX  0x008A
#define MB_REG_RANGE_MY_MIN  0x008C
#define MB_REG_RANGE_MY_MAX  0x008E
#define MB_REG_RANGE_MZ_MIN  0x0090
#define MB_REG_RANGE_MZ_MAX  0x0092

/* ---- Network Config (R/W) --------------------------------------- */
#define MB_REG_MAC           0x0100  /* MAC,  3 words effective (6 regs)  */
#define MB_REG_IP            0x0106  /* IP,   2 words                     */
#define MB_REG_SUBNET        0x0108  /* Subnet,2 words                    */
#define MB_REG_GATEWAY       0x010A  /* Gateway,2 words                   */

/* ---- Extended Registers (internal, NOT in customer doc) --------- */
#define MB_REG_FREQ_MODE     0x0200  /* 0=500Hz, 1=1000Hz                */

/* ---- Self-Test Status (read-only, extended) --------------------- */
#define MB_REG_ST_ERROR      0x0210  /* Error flags word 0 (uint16)       */
#define MB_REG_ST_ERROR_HI   0x0211  /* Error flags word 1 (uint16)       */
#define MB_REG_ST_ADC_ID     0x0212  /* ADC chip ID + ERROR byte          */
#define MB_REG_ST_W5500      0x0213  /* W5500 version + PHY link + speed  */
#define MB_REG_ST_RS485      0x0214  /* RS485 UART + DMA status           */
#define MB_REG_ST_FLASH      0x0215  /* Flash valid + matrix + zero + cfg */

/* ---- Block boundaries (for valid_range grouped checks) ----------
 * IMPORTANT: `_END` is the INCLUSIVE last register address, and it must be
 * exactly covered by the matching `case` range in mb_reg_read(). A block end
 * that reaches past the case range makes valid_range() pass while the per-word
 * switch falls through to `default` and answers exception 0x02.
 * ----------------------------------------------------------------- */
#define MB_BLK_TRIG_START    0x0001
#define MB_BLK_TRIG_END      0x0003

#define MB_BLK_INFO_START    0x0005
#define MB_BLK_INFO_END      0x0010

#define MB_BLK_CTRL_START    0x0030
#define MB_BLK_CTRL_END      0x003E      /* ZERO + UNZERO + FORMAT + FORCE ×6
                                          * (0x33..0x3E = 12 regs, MZ high word
                                          *  0x3E included)                  */

#define MB_BLK_ZERO_START    0x0050
#define MB_BLK_ZERO_END      0x005B

#define MB_BLK_OVL_START     0x0070
#define MB_BLK_OVL_END       0x007B

#define MB_BLK_RANGE_START   0x007C
#define MB_BLK_RANGE_END     0x0093      /* 6 axes × (min+max) × 2 words
                                          * (0x7C..0x93 = 24 regs, MZ_MAX high
                                          *  word 0x93 included)             */

#define MB_BLK_NET_START     0x0100
#define MB_BLK_NET_END       0x010B

#define MB_BLK_EXT_START     0x0200
#define MB_BLK_EXT_END       0x0200      /* only FREQ_MODE for now */

#define MB_BLK_ST_START      0x0210
#define MB_BLK_ST_END        0x0215

/* ---- Function / Exception Codes -------------------------------- */
#define MB_FC_READ_HOLDING   0x03
#define MB_FC_WRITE_SINGLE   0x06
#define MB_FC_WRITE_MULTI    0x10

#define MB_EX_NONE           0x00
#define MB_EX_ILLEGAL_FC     0x01
#define MB_EX_ILLEGAL_ADDR   0x02
#define MB_EX_ILLEGAL_DATA   0x03
#define MB_EX_ILLEGAL_ACTION 0x04

/* ================================================================
 *  Register Access API
 *
 *  These functions are called by the Modbus-TCP protocol layer.
 *  They return the number of bytes read/written, or 0 on error
 *  (exception code is written to *exc).
 * ================================================================ */

/** Read holding registers: buf must hold at least count*2 bytes */
uint16_t mb_reg_read(uint16_t addr, uint16_t count,
                     uint8_t *buf, uint8_t *exc);

/** Write single register */
uint16_t mb_reg_write_single(uint16_t addr, uint16_t value, uint8_t *exc);

/** Write multiple registers: data contains count*2 bytes */
uint16_t mb_reg_write_multi(uint16_t addr, uint16_t count,
                            const uint8_t *data, uint8_t *exc);

#ifdef __cplusplus
}
#endif

#endif /* __MB_REGISTER_H__ */
