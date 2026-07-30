/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w5500.h"
#include "usart.h"
#include <string.h>
#include "sensor_types.h"
#include "modbus_tcp.h"
#include "rs485.h"
#include "rs485_cmd.h"
#include "flash_storage.h"
#include "calibration.h"
#include "lha7668.h"
#include "self_test.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SAMPLING_RATE_0       0
#define ADC_SAMPLING_RATE_3       3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t g_tick_ms = 0;

/* Shared global state (declared in sensor_types.h) */
volatile sensor_data_t   g_sensor;
volatile system_state_t  g_sys;
config_t                 g_config = {
    .mac        = {0x00,0x08,0xDC,0x01,0x02,0x03},
    .ip         = {192,168,1,12},
    .subnet     = {255,255,255,0},
    .gateway    = {192,168,1,1},
    .sn         = "HS-01234567",
    .fw_version = 0x0100,
};
decouple_matrix_t        g_matrix;
threshold_data_t         g_threshold; /* overload/range thresholds (not in Flash) */
lhl_lha7668_ctx_t lha7668_ctx = { 0 };  /* LHA7668B official driver handle */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t int_adcData(void);

#define ADC_DATA_BUFF_LEN       64
#define ADC_CHANNEL_NUM         6
#define KILOGRAM_TO_NEWTON      9.8

uint8_t output_interface = 0;
uint8_t socket_connect_num[4] = { 0 };

static uint8_t adc_data_int_count = 0;
static uint8_t adc_data_usr_count = 0;
static uint32_t adc_data_buff[ADC_DATA_BUFF_LEN][6] = { 0 };

static float adc_data_calculate(uint8_t index)
{
    return (float)((int32_t)adc_data_buff[adc_data_usr_count][index] - 0x800000) * (1.0f / 8388608.0f) * 2500;
}

float matrix[6][6] = { {0.298418, -0.211604, -0.529767, -0.012232, -0.178473, 0.211071 },
                        {0.004134, 0.211038, -0.006878, 0.007002, -0.001183, -0.001670 },
                        {0.006217, -0.000427, 0.558141, 0.003760, -0.004378, -0.142885 },
                        {-0.000174, -0.000562, 0.001114, 0.001136, 0.000010, -0.000292 },
                        {-0.000243, -0.000483, -0.003015, -0.000066, -0.001470, 0.001025 },
                        {0.000317, -0.000204, -0.000423, -0.000031, -0.000158, 0.001769 }};

static void adc_data_proc(float data[6])
{
    // int i = 0;
    // if (g_sys.data_format == 0) {
    //     for (i = 0; i < ADC_CHANNEL_NUM; i++) {
    //         g_sensor.force[i] = data[i];
    //     }
    // } else if (g_sys.data_format == 1) {
    //     g_sensor.force[0] = matrix[0][0] * data[0] / KILOGRAM_TO_NEWTON + matrix[0][1] * data[1] / KILOGRAM_TO_NEWTON +
    //                 matrix[0][2] * data[2] / KILOGRAM_TO_NEWTON + matrix[0][3] * data[3] / KILOGRAM_TO_NEWTON +
    //                 matrix[0][4] * data[4] / KILOGRAM_TO_NEWTON + matrix[0][5] * data[5] / KILOGRAM_TO_NEWTON;

    //     g_sensor.force[1] = matrix[1][0] * data[0] / KILOGRAM_TO_NEWTON + matrix[1][1] * data[1] / KILOGRAM_TO_NEWTON +
    //                 matrix[1][2] * data[2] / KILOGRAM_TO_NEWTON + matrix[1][3] * data[3] / KILOGRAM_TO_NEWTON +
    //                 matrix[1][4] * data[4] / KILOGRAM_TO_NEWTON + matrix[1][5] * data[5] / KILOGRAM_TO_NEWTON;

    //     g_sensor.force[2] = matrix[2][0] * data[0] / KILOGRAM_TO_NEWTON + matrix[2][1] * data[1] / KILOGRAM_TO_NEWTON +
    //                 matrix[2][2] * data[2] / KILOGRAM_TO_NEWTON + matrix[2][3] * data[3] / KILOGRAM_TO_NEWTON +
    //                 matrix[2][4] * data[4] / KILOGRAM_TO_NEWTON + matrix[2][5] * data[5] / KILOGRAM_TO_NEWTON;

    //     g_sensor.force[3] = matrix[3][0] * data[0] / KILOGRAM_TO_NEWTON + matrix[3][1] * data[1] / KILOGRAM_TO_NEWTON +
    //                 matrix[3][2] * data[2] / KILOGRAM_TO_NEWTON + matrix[3][3] * data[3] / KILOGRAM_TO_NEWTON +
    //                 matrix[3][4] * data[4] / KILOGRAM_TO_NEWTON + matrix[3][5] * data[5] / KILOGRAM_TO_NEWTON;

    //     g_sensor.force[4] = matrix[4][0] * data[0] / KILOGRAM_TO_NEWTON + matrix[4][1] * data[1] / KILOGRAM_TO_NEWTON +
    //                 matrix[4][2] * data[2] / KILOGRAM_TO_NEWTON + matrix[4][3] * data[3] / KILOGRAM_TO_NEWTON +
    //                 matrix[4][4] * data[4] / KILOGRAM_TO_NEWTON + matrix[4][5] * data[5] / KILOGRAM_TO_NEWTON;

    //     g_sensor.force[5] = matrix[5][0] * data[0] / KILOGRAM_TO_NEWTON + matrix[5][1] * data[1] / KILOGRAM_TO_NEWTON +
    //                 matrix[5][2] * data[2] / KILOGRAM_TO_NEWTON + matrix[5][3] * data[3] / KILOGRAM_TO_NEWTON +
    //                 matrix[5][4] * data[4] / KILOGRAM_TO_NEWTON + matrix[5][5] * data[5] / KILOGRAM_TO_NEWTON;
    // } else if (g_sys.data_format == 2) {
    //     g_sensor.force[0] = matrix[0][0] * data[0] + matrix[0][1] * data[1] +
    //                 matrix[0][2] * data[2] + matrix[0][3] * data[3] +
    //                 matrix[0][4] * data[4] + matrix[0][5] * data[5];

    //     g_sensor.force[1] = matrix[1][0] * data[0] + matrix[1][1] * data[1] +
    //                 matrix[1][2] * data[2] + matrix[1][3] * data[3] +
    //                 matrix[1][4] * data[4] + matrix[1][5] * data[5];

    //     g_sensor.force[2] = matrix[2][0] * data[0] + matrix[2][1] * data[1] +
    //                 matrix[2][2] * data[2] + matrix[2][3] * data[3] +
    //                 matrix[2][4] * data[4] + matrix[2][5] * data[5];

    //     g_sensor.force[3] = matrix[3][0] * data[0] + matrix[3][1] * data[1] +
    //                 matrix[3][2] * data[2] + matrix[3][3] * data[3] +
    //                 matrix[3][4] * data[4] + matrix[3][5] * data[5];

    //     g_sensor.force[4] = matrix[4][0] * data[0] + matrix[4][1] * data[1] +
    //                 matrix[4][2] * data[2] + matrix[4][3] * data[3] +
    //                 matrix[4][4] * data[4] + matrix[4][5] * data[5];

    //     g_sensor.force[5] = matrix[5][0] * data[0] + matrix[5][1] * data[1] +
    //                 matrix[5][2] * data[2] + matrix[5][3] * data[3] +
    //                 matrix[5][4] * data[4] + matrix[5][5] * data[5];
    // }
    int i = 0;
    if (g_sys.data_format == 0) {
        for (i = 0; i < ADC_CHANNEL_NUM; i++) {
            g_sensor.force[i] = data[i];
        }
    } else if (g_sys.data_format == 1) {
        g_sensor.force[0] = g_matrix.m[0][0] * data[0] / KILOGRAM_TO_NEWTON + g_matrix.m[0][1] * data[1] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[0][2] * data[2] / KILOGRAM_TO_NEWTON + g_matrix.m[0][3] * data[3] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[0][4] * data[4] / KILOGRAM_TO_NEWTON + g_matrix.m[0][5] * data[5] / KILOGRAM_TO_NEWTON;

        g_sensor.force[1] = g_matrix.m[1][0] * data[0] / KILOGRAM_TO_NEWTON + g_matrix.m[1][1] * data[1] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[1][2] * data[2] / KILOGRAM_TO_NEWTON + g_matrix.m[1][3] * data[3] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[1][4] * data[4] / KILOGRAM_TO_NEWTON + g_matrix.m[1][5] * data[5] / KILOGRAM_TO_NEWTON;

        g_sensor.force[2] = g_matrix.m[2][0] * data[0] / KILOGRAM_TO_NEWTON + g_matrix.m[2][1] * data[1] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[2][2] * data[2] / KILOGRAM_TO_NEWTON + g_matrix.m[2][3] * data[3] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[2][4] * data[4] / KILOGRAM_TO_NEWTON + g_matrix.m[2][5] * data[5] / KILOGRAM_TO_NEWTON;

        g_sensor.force[3] = g_matrix.m[3][0] * data[0] / KILOGRAM_TO_NEWTON + g_matrix.m[3][1] * data[1] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[3][2] * data[2] / KILOGRAM_TO_NEWTON + g_matrix.m[3][3] * data[3] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[3][4] * data[4] / KILOGRAM_TO_NEWTON + g_matrix.m[3][5] * data[5] / KILOGRAM_TO_NEWTON;

        g_sensor.force[4] = g_matrix.m[4][0] * data[0] / KILOGRAM_TO_NEWTON + g_matrix.m[4][1] * data[1] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[4][2] * data[2] / KILOGRAM_TO_NEWTON + g_matrix.m[4][3] * data[3] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[4][4] * data[4] / KILOGRAM_TO_NEWTON + g_matrix.m[4][5] * data[5] / KILOGRAM_TO_NEWTON;

        g_sensor.force[5] = g_matrix.m[5][0] * data[0] / KILOGRAM_TO_NEWTON + g_matrix.m[5][1] * data[1] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[5][2] * data[2] / KILOGRAM_TO_NEWTON + g_matrix.m[5][3] * data[3] / KILOGRAM_TO_NEWTON +
                    g_matrix.m[5][4] * data[4] / KILOGRAM_TO_NEWTON + g_matrix.m[5][5] * data[5] / KILOGRAM_TO_NEWTON;
    } else if (g_sys.data_format == 2) {
        g_sensor.force[0] = g_matrix.m[0][0] * data[0] + g_matrix.m[0][1] * data[1] +
                    g_matrix.m[0][2] * data[2] + g_matrix.m[0][3] * data[3] +
                    g_matrix.m[0][4] * data[4] + g_matrix.m[0][5] * data[5];

        g_sensor.force[1] = g_matrix.m[1][0] * data[0] + g_matrix.m[1][1] * data[1] +
                    g_matrix.m[1][2] * data[2] + g_matrix.m[1][3] * data[3] +
                    g_matrix.m[1][4] * data[4] + g_matrix.m[1][5] * data[5];

        g_sensor.force[2] = g_matrix.m[2][0] * data[0] + g_matrix.m[2][1] * data[1] +
                    g_matrix.m[2][2] * data[2] + g_matrix.m[2][3] * data[3] +
                    g_matrix.m[2][4] * data[4] + g_matrix.m[2][5] * data[5];

        g_sensor.force[3] = g_matrix.m[3][0] * data[0] + g_matrix.m[3][1] * data[1] +
                    g_matrix.m[3][2] * data[2] + g_matrix.m[3][3] * data[3] +
                    g_matrix.m[3][4] * data[4] + g_matrix.m[3][5] * data[5];

        g_sensor.force[4] = g_matrix.m[4][0] * data[0] + g_matrix.m[4][1] * data[1] +
                    g_matrix.m[4][2] * data[2] + g_matrix.m[4][3] * data[3] +
                    g_matrix.m[4][4] * data[4] + g_matrix.m[4][5] * data[5];

        g_sensor.force[5] = g_matrix.m[5][0] * data[0] + g_matrix.m[5][1] * data[1] +
                    g_matrix.m[5][2] * data[2] + g_matrix.m[5][3] * data[3] +
                    g_matrix.m[5][4] * data[4] + g_matrix.m[5][5] * data[5];
    }

    for (i = 0; i < ADC_CHANNEL_NUM; i++) {
        g_sensor.force[i] = FloatFilter_UpdateChannel(i, g_sensor.force[i]);
    }
}

/* W5500 callback: TIM2 period elapsed -> increment tick counter */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        g_tick_ms++;
        
        if (g_tick_ms % 3 == 0) {
            adc_data_buff[adc_data_int_count][0] = int_adcData();
            adc_data_buff[adc_data_int_count][1] = int_adcData();
            adc_data_buff[adc_data_int_count][2] = int_adcData();
            adc_data_buff[adc_data_int_count][3] = int_adcData();
            adc_data_buff[adc_data_int_count][4] = int_adcData();
            adc_data_buff[adc_data_int_count][5] = int_adcData();
            adc_data_int_count++;
            if (adc_data_int_count >= ADC_DATA_BUFF_LEN) {
                adc_data_int_count = 0;
            }
        }
    }
}

/* W5500 network parameters — fetched from g_config (loaded from Flash or defaults) */

#define W5500_PORT        502

/* Per-socket connection context */
typedef struct {
    uint8_t  connected;        /* 1 = ESTABLISHED */
    uint32_t last_tick;        /* 上次发送时间戳 (ms) */
} sock_ctx_t;

static void uart_debug(const char *msg)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, (uint16_t)strlen(msg), 100);
}

static void uart_debug_ip(const char *prefix, const uint8_t *ip)
{
    /* Print prefix + dotted-decimal IP without snprintf dependency */
    static const char dots[] = ".";
    static const char crlf[] = "\r\n";
    char num[4];
    uint8_t pos;

    HAL_UART_Transmit(&huart2, (const uint8_t *)prefix, (uint16_t)strlen(prefix), 100);
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t v = ip[i];
        pos = 0;
        if (v >= 100) { num[pos++] = '0' + (v / 100); v %= 100; }
        if (v >= 10)  { num[pos++] = '0' + (v / 10);  v %= 10;  }
        num[pos++] = '0' + v;
        num[pos] = '\0';
        HAL_UART_Transmit(&huart2, (uint8_t *)num, pos, 100);
        HAL_UART_Transmit(&huart2, (const uint8_t *)((i < 3) ? dots : crlf),
                          (uint16_t)((i < 3) ? 1 : 2), 100);
    }
}

static void uart_debug_hex8(const char *prefix, uint8_t val)
{
    char buf[5];
    buf[0] = "0123456789ABCDEF"[val >> 4];
    buf[1] = "0123456789ABCDEF"[val & 0x0F];
    buf[2] = '\r';
    buf[3] = '\n';
    buf[4] = '\0';
    HAL_UART_Transmit(&huart2, (const uint8_t *)prefix, (uint16_t)strlen(prefix), 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, 4, 100);
}

void LHA7668_Platform_Init(void *handle)
{
    lhl_lha7668_ctx_t *dev_ctx = (lhl_lha7668_ctx_t *)handle;
    dev_ctx->io.cs.port = GPIOB;
    dev_ctx->io.cs.pin = GPIO_PIN_12;
}

void LHA7668_Platform_Set(const void *port, const uint32_t pin)
{
    HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin, GPIO_PIN_SET);
}

void LHA7668_Platform_Reset(const void *port, const uint32_t pin)
{
    HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin, GPIO_PIN_RESET);
}

int32_t LHA7668_Platform_ReadWrite(uint8_t *txdata, uint8_t *rxdata, const uint16_t size)
{
    HAL_SPI_TransmitReceive(&hspi2, txdata, rxdata, size, 0xFFFF);

    return 0;
}

static uint32_t int_adcData(void)
{
    LHL_LHA7668_Start(&lha7668_ctx, LHA7668_MODE_SINGLE_SHOT);

    while (LHL_LHA7668_Get_Flag(&lha7668_ctx, LHA7668_STATUS_RDY_FLAG) == LHA7668_SET);

    LHL_LHA7668_Get_Data(&lha7668_ctx);

    return lha7668_ctx.data;
}

float adcData(void)
{
    LHL_LHA7668_Start(&lha7668_ctx, LHA7668_MODE_SINGLE_SHOT);

    while (LHL_LHA7668_Get_Flag(&lha7668_ctx, LHA7668_STATUS_RDY_FLAG) == LHA7668_SET);

    LHL_LHA7668_Get_Data(&lha7668_ctx);

    return LHL_LHA7668_Get_mVoltage(lha7668_ctx.data, LHA7668_BIPOLAR, LHA7668_PGA_X128, 2500);
}

void lha7668_init(uint8_t rate)
{
    LHL_LHA7668_Init(&lha7668_ctx);
    LHL_LHA7668_Reset(&lha7668_ctx);

    uint8_t whoamI = LHL_LHA7668_Get_ID(&lha7668_ctx);
    if (whoamI != LHA7668B_8) {
        uart_debug("Equipment error!\r\n");
        while(1) {
        }
    }

    LHL_LHA7668_Stop(&lha7668_ctx);

    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN0;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN1;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_0);
    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN2;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN3;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_1);
    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN4;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN5;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_2);
    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN6;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN7;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_3);
    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN8;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN9;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_4);
    lha7668_ctx.CHANNEL.ENABLE = LHA7668_ENABLE;
    lha7668_ctx.CHANNEL.SETUP = LHA7668_SETUP_0;
    lha7668_ctx.CHANNEL.AINP = LHA7668_AIN10;
    lha7668_ctx.CHANNEL.AINM = LHA7668_AIN11;
    LHL_LHA7668_Set_Channel(&lha7668_ctx, LHA7668_CHANNEL_5);

    lha7668_ctx.SETUP.BIPOLAR = LHA7668_BIPOLAR;
    lha7668_ctx.SETUP.AIN_BUFP = LHA7668_ENABLE;
    lha7668_ctx.SETUP.AIN_BUFM = LHA7668_ENABLE;
    lha7668_ctx.SETUP.REF_SEL = LHA7668_REF_REFIN1;
    lha7668_ctx.SETUP.REF_BUFP = LHA7668_DISABLE;
    lha7668_ctx.SETUP.REF_BUFM = LHA7668_DISABLE;
    lha7668_ctx.SETUP.PGA = LHA7668_PGA_X128;
    lha7668_ctx.SETUP.FILTER = LHA7668_FILTER_SINC3;
    lha7668_ctx.SETUP.FS = rate;
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_0);
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_1);
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_2);
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_3);
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_4);
    LHL_LHA7668_Set_Setup(&lha7668_ctx, LHA7668_SETUP_5);
    lha7668_ctx.ADC_CTRL.POWER_MODE = LHA7668_FULL_POWER;
    lha7668_ctx.ADC_CTRL.CLK_SEL = LHA7668_CLK_INTL;
    lha7668_ctx.ADC_CTRL.DATA_STATUS = LHA7668_ENABLE;
    lha7668_ctx.ADC_CTRL.CS_EN = LHA7668_DISABLE;
    lha7668_ctx.ADC_CTRL.DOUT_RDY_DEL = LHA7668_DISABLE;
    LHL_LHA7668_Set_ADC(&lha7668_ctx);
}

static sock_ctx_t g_sock[W5500_SOCK_COUNT];

static void w5500_network_init(void)
{
    uart_debug("--- W5500 Network Config ---\r\n");

    /* Configure network */
    uart_debug("Configuring network: IP=...\r\n");
    w5500_set_mac(g_config.mac);
    w5500_set_ip(g_config.ip);
    w5500_set_gateway(g_config.gateway);
    w5500_set_subnet(g_config.subnet);

    /* Read back IP for verification */
    uint8_t ip_check[4];
    w5500_read_buf(W5500_BS_COMMON, W5500_SIPR, ip_check, 4);
    uart_debug_ip("IP readback: ", ip_check);

    /* Wait for PHY link (auto-negotiation takes 1-3 seconds) */
    uart_debug("Waiting for PHY link...\r\n");
    {
        uint32_t tout = 3000;
        uint8_t  link_up = 0;
        while (tout > 0) {
            uint8_t phy = w5500_read_byte(W5500_BS_COMMON, W5500_PHYCFGR);
            if (phy & 0x01) { link_up = 1; break; }
            HAL_Delay(10);
            tout -= 10;
        }
        uart_debug(link_up ? "PHY link UP\r\n" : "PHY link DOWN (continuing...)\r\n");
    }

    /* Configure ALL 8 socket buffers at once (W5500 batch-allocation requirement) */
    w5500_configure_buffers();

    /* Initialize all used sockets — listen on port 502 for multi-client support */
    static const char *init_fail[] = {"S0 init FAIL SR=","S1 init FAIL SR=","S2 init FAIL SR=","S3 init FAIL SR="};
    static const char *listen_fail[] = {"S0 listen FAIL SR=","S1 listen FAIL SR=","S2 listen FAIL SR=","S3 listen FAIL SR="};
    for (uint8_t i = 0; i < W5500_SOCK_COUNT; i++) {
        g_sock[i].connected = 0;
        g_sock[i].last_tick = 0;
        uint8_t sr = w5500_socket_init(i, W5500_PORT);
        if (sr != SOCK_INIT) {
            uart_debug_hex8(init_fail[i], sr);
            Error_Handler();
        }
        sr = w5500_socket_listen(i);
        if (sr != SOCK_LISTEN) {
            uart_debug_hex8(listen_fail[i], sr);
            Error_Handler();
        }
    }
    uart_debug("All sockets listening on port 502\r\n");
    uart_debug("--- W5500 Init Done ---\r\n");
}

/* ---- Modbus-TCP send callback (bridges modbus_tcp.c → w5500) ---- */
static void mb_tcp_send_cb(uint8_t sock, const uint8_t *data,
                           uint16_t len, void *ctx)
{
    (void)ctx;
    w5500_socket_send(sock, data, len);
}

/* ---- USART2 IDLE interrupt: feed DMA bytes into RS485 ring buffer ---- */
/*     Called before HAL_UART_IRQHandler to ensure CNDTR is read correctly */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        if (huart2.hdmarx) {
            uint16_t rem = (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);
            rs485_rx_idle_cb(rem);
        }
    }
    HAL_UART_IRQHandler(&huart2);
}

/* ---- Continuous RS485 data send state ---- */
static uint32_t g_last_rs485_send_ms;

void send_string(uint8_t *str, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, len, 500);
}

uint8_t hex_buf_send[256] = { 0 };
uint8_t mb_buf_send[33] = { 0 };

#define HEX_BUF_SEND_OFFSET 5

static void rs485_send_continuous(void)
{
    uint32_t interval = (g_sys.freq_mode == 0) ? 2U : 1U;  /* 500Hz or 1kHz */
    if ((g_tick_ms - g_last_rs485_send_ms) >= interval) {
        g_last_rs485_send_ms = g_tick_ms;

        if (g_sys.debug_mode == 0) {
            if (output_interface == 1) {
                /* Build data frame: AA 55 00 [SEQ4B] [6×float32] 0D 0A */
                uint32_t seq = g_sys.frame_seq++;
                hex_buf_send[0] = RS485_CMD_CONT_DATA;
                hex_buf_send[1] = (uint8_t)(seq);
                hex_buf_send[2] = (uint8_t)(seq >> 8);
                hex_buf_send[3] = (uint8_t)(seq >> 16);
                hex_buf_send[4] = (uint8_t)(seq >> 24);
                for (int i = 0; i < 6; i++) {
                    float f = g_sensor.force[i];   /* read volatile once */
                    uint32_t u; memcpy(&u, &f, 4);
                    hex_buf_send[HEX_BUF_SEND_OFFSET + i * 4 + 0] = (uint8_t)(u);
                    hex_buf_send[HEX_BUF_SEND_OFFSET + i * 4 + 1] = (uint8_t)(u >> 8);
                    hex_buf_send[HEX_BUF_SEND_OFFSET + i * 4 + 2] = (uint8_t)(u >> 16);
                    hex_buf_send[HEX_BUF_SEND_OFFSET + i * 4 + 3] = (uint8_t)(u >> 24);
                }

                rs485_send_raw(hex_buf_send, 29);
            } else if (output_interface == 2) {
                for (int i = 0; i < 6; i++) {
                    float f = g_sensor.force[i];   /* read volatile once */
                    uint32_t u; memcpy(&u, &f, 4);
                    mb_buf_send[9 + i*4 + 0] = (uint8_t)(u);
                    mb_buf_send[9 + i*4 + 1] = (uint8_t)(u >> 8);
                    mb_buf_send[9 + i*4 + 2] = (uint8_t)(u >> 16);
                    mb_buf_send[9 + i*4 + 3] = (uint8_t)(u >> 24);
                }
                for (uint8_t j = 0; j < 4; j++) {
                    if (socket_connect_num[j] == 1) {
                        mb_buf_send[0] = 0x00;
                        mb_buf_send[1] = 0x00;
                        mb_buf_send[2] = 0x00;
                        mb_buf_send[3] = 0x00;
                        mb_buf_send[4] = 0x00;
                        mb_buf_send[5] = 0x1B;
                        mb_buf_send[6] = 0x01;
                        mb_buf_send[7] = 0x03;
                        mb_buf_send[8] = 0x18;
                        w5500_socket_send(j, mb_buf_send, 33);
                    }
                }
            }
        } else if (g_sys.debug_mode == 1) {
            for (int i = 0; i < 6; i++) {
                float f = g_sensor.force[i];   /* read volatile once */
                uint32_t u; memcpy(&u, &f, 4);
                hex_buf_send[i * 4 + 0] = (uint8_t)(u);
                hex_buf_send[i * 4 + 1] = (uint8_t)(u >> 8);
                hex_buf_send[i * 4 + 2] = (uint8_t)(u >> 16);
                hex_buf_send[i * 4 + 3] = (uint8_t)(u >> 24);
            }
            hex_buf_send[24] = 0x00;
            hex_buf_send[25] = 0x00;
            hex_buf_send[26] = 0x80;
            hex_buf_send[27] = 0x7F;
            just_float_send_raw(hex_buf_send, 28);
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* Load persistent config / matrix / zero from Flash */
    if (!flash_load_all()) {
        uart_debug("Flash empty — using NaN placeholders\r\n");
        /* Flash is empty / never written → fill matrix & zero with NaN
        * so all force outputs become NaN, clearly distinguishing
        * "uncalibrated" from "zero load" (which would output 0.0). */
        static const uint32_t nan_bits = 0x7FC00000U;
        float nan_val;
        memcpy(&nan_val, &nan_bits, 4);
        for (int i = 0; i < 6; i++) {
            g_sensor.force_zero[i] = nan_val;
            g_threshold.overload[i]  = nan_val;
            g_threshold.range_min[i] = nan_val;
            g_threshold.range_max[i] = nan_val;
            for (int j = 0; j < 6; j++) {
                g_matrix.m[i][j] = nan_val;
            }
        }
    } else {
        uart_debug("Flash config loaded OK\r\n");
    }
    calib_init();
    g_sys.data_format = 0;
    FloatFilter_Init();

    /* ================================================================
    *  BOOT SELF-TEST
    *  Check ADC, W5500, RS485, Flash in sequence.
    *  Results are stored in g_self_test and available via Modbus
    *  registers (MB_REG_ST_ERROR ~ MB_REG_ST_FLASH).
    * ================================================================ */
    uart_debug("\r\n>>> BOOT SELF-TEST START <<<\r\n");

    uart_debug("Resetting W5500...\r\n");
    if (!w5500_init()) {
        uart_debug("ERROR: W5500 not detected!\r\n");
    } else {
        uart_debug("W5500 OK (ver=0x04)\r\n");
    }

    /* Bind platform callbacks: SPI2, PB12(CS) */
    lha7668_init(ADC_SAMPLING_RATE_3);
    uart_debug("LHA7668 OK\r\n");

    /* 2. W5500 Ethernet self-test (SPI + version + PHY) */
    uart_debug("--- W5500 Ethernet self-test ---\r\n");
    uint32_t st_w5500 = self_test_w5500();
    if (st_w5500 == ST_ERR_NONE) {
        uart_debug("  W5500 SPI OK, ver=0x");
        uart_debug_hex8("", g_self_test.w5500_version);
    } else {
        uart_debug("  W5500 FAIL: flags=0x");
        uart_debug_hex8("", (uint8_t)(st_w5500 >> 8));
        uart_debug_hex8("", (uint8_t)(st_w5500));
        uart_debug("\r\n");
    }

    /* 3. RS485 self-test (USART + DMA) */
    uart_debug("--- RS485 self-test ---\r\n");
    uint32_t st_rs485 = self_test_rs485();
    uart_debug(st_rs485 == ST_ERR_NONE ?
        "  RS485 OK (USART+DMA)\r\n" :
        "  RS485 FAIL\r\n");

    /* 4. Flash storage self-test (magic + CRC + data validation) */
    uart_debug("--- Flash storage self-test ---\r\n");
    uint32_t st_flash = self_test_flash();
    if (st_flash == ST_ERR_NONE) {
        uart_debug("  Flash OK (magic+CRC+data)\r\n");
    } else {
        uart_debug("  Flash FAIL: ");
        if (st_flash & ST_ERR_FLASH_EMPTY) uart_debug("EMPTY ");
        if (st_flash & ST_ERR_FLASH_CRC)   uart_debug("CRC ");
        uart_debug("\r\n");
    }

    /* Print full structured report */
    self_test_print_report(&huart2);

    w5500_network_init();
    HAL_TIM_Base_Start_IT(&htim2);   /* Start 1kHz timer */
    rs485_init(&huart2, rs485_cmd_dispatch);
    g_last_rs485_send_ms = g_tick_ms;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        static uint32_t last_beat = 0;

        /* ---- Heartbeat ---- */
        if ((g_tick_ms - last_beat) >= 5000) {
            uart_debug(".\r\n");
            last_beat = g_tick_ms;
        }

        /* ================================================================
        *  ADC data acquisition (LHA7668B, non-blocking per-channel read)
        *
        *  The sequencer auto-cycles CH0→CH5 at ~200 SPS/ch (FS=0).
        *  We read one channel per loop iteration (DRDY-polling).
        *  When all 6 channels are updated, push through the calibration
        *  pipeline: raw mV → 20pt filter → U → F=K×U → F_zero subtract.
        * ================================================================ */
        {
            // static int   adc_sample_count = 0;
            float adc_tmp[ADC_CHANNEL_NUM] = { 0 };

            // /* Poll DRDY pin (PB13) — low = data ready */
            // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
            //     uint32_t raw = 0;
            //     uint8_t  ch  = 0;
            //     ch &= LHA7668_STATUS_CH_ACTIVE_MSK;

            //     if (ch < 6) {
            //         /* Sign-extend 24-bit two's complement → int32 */
            //         int32_t code;
            //         uint32_t raw24 = raw & 0x00FFFFFFu;
            //         if (raw24 & 0x00800000u)
            //         code = (int32_t)(raw24 | 0xFF000000u);
            //         else
            //         code = (int32_t)raw24;
            //         float mv = (float)code * (2500.0f / 8388608.0f);

            //         g_sensor.force_raw[ch] = mv;
            //         adc_sample_count++;

            //         if (adc_sample_count >= 6) {
            //             adc_sample_count = 0;
            //             /* Run full pipeline + save filtered mV for format=0 */
            //             calib_process_sample_with_mv((const float *)g_sensor.force_raw,
            //                                 (float *)g_sensor.force);

            //             /* Apply data-format conversion to g_sensor.force[]
            //             *   format=0 (mV)  → copy filtered mV (raw bridge voltage)
            //             *   format=1 (kg)  → N / 9.80665  (kg for force, kgm for torque)
            //             *   format=2 (N)   → leave as N/Nm (already correct)
            //             * Both RS485 continuous & Modbus-TCP read from g_sensor.force[] */
            //             if (g_sys.data_format == 0) {
            //                 /* mV mode: output filtered mV from ADC directly */
            //                 for (int i = 0; i < 6; i++)
            //                 g_sensor.force[i] = g_sensor.force_mv[i];
            //             } else if (g_sys.data_format == 1) {
            //                 /* kg / kgm mode: convert from N to kg */
            //                 static const float G = 9.80665f;
            //                 for (int i = 0; i < 6; i++)
            //                 g_sensor.force[i] /= G;
            //             }
            //             /* format=2 (N/Nm): no conversion needed */
            //         }
            //     }
            // }

            if (adc_data_usr_count != adc_data_int_count) {
                adc_data_usr_count = adc_data_int_count;
                for (uint8_t i = 0; i < ADC_CHANNEL_NUM; i++) {
                    adc_tmp[i] = adc_data_calculate(i);

                    if (g_sys.zero_calib_busy == 0) {
                        adc_tmp[i] -= g_sensor.force_zero[i];
                    }
                }

                if (g_sys.zero_calib_busy == 1) {
                    calib_zero_cale(adc_tmp);
                }

                adc_data_proc(adc_tmp);
            }
        }

        /* ---- Zero calibration: auto-detect completion & persist ---- */
        if (calib_zero_is_done()) {
            // uint8_t buf[28] = { 0 };
            // for (int i = 0; i < 6; i++) {
            //     float f = g_sensor.force[i];   /* read volatile once */
            //     uint32_t u; memcpy(&u, &f, 4);
            //     buf[2 + i * 4 + 0] = (uint8_t)(u);
            //     buf[2 + i * 4 + 1] = (uint8_t)(u >> 8);
            //     buf[2 + i * 4 + 2] = (uint8_t)(u >> 16);
            //     buf[2 + i * 4 + 3] = (uint8_t)(u >> 24);
            // }
            // buf[0] = 0xBB;
            // buf[1] = 0xCC;
            // buf[26] = 0xDD;
            // buf[27] = 0xEE;
            // rs485_send_raw(buf, 28);

            g_sys.zero_calib_busy = 0;
            calib_zero_over();
            flash_save_zero();
            uart_debug("Zero calib done — saved to Flash\r\n");
        }

        /* ================================================================
        *  RS485: continuous data output  (send_mode == 1)
        *         single-shot output      (send_mode == 2)
        * ================================================================ */
        if (g_sys.send_mode == 1) {
            rs485_send_continuous();
        } else if (g_sys.send_mode == 2) {
            /* Single-shot: send one data frame then stop */
            uint8_t buf[31];
            uint32_t seq = g_sys.frame_seq++;
            buf[0] = RS485_CMD_CONT_DATA;
            buf[1] = (uint8_t)(seq);
            buf[2] = (uint8_t)(seq >> 8);
            buf[3] = (uint8_t)(seq >> 16);
            buf[4] = (uint8_t)(seq >> 24);
            for (int i = 0; i < 6; i++) {
                float f = g_sensor.force[i];
                uint32_t u; memcpy(&u, &f, 4);
                buf[5 + i*4 + 0] = (uint8_t)(u);
                buf[5 + i*4 + 1] = (uint8_t)(u >> 8);
                buf[5 + i*4 + 2] = (uint8_t)(u >> 16);
            }
            rs485_send_raw(buf, 29);
            g_sys.send_mode = 0;   /* auto-reset after single shot */
        } else {
            g_last_rs485_send_ms = g_tick_ms;   /* reset timer on stop */
        }

        /* ================================================================
        *  RS485: frame scanner (command handling)
        * ================================================================ */
        rs485_process();

        /* ================================================================
        *  Modbus-TCP: W5500 socket management + protocol processing
        * ================================================================ */
        static uint8_t last_sr[W5500_SOCK_COUNT] = {0};
        static uint8_t sock_inited[W5500_SOCK_COUNT] = {0};

        for (uint8_t i = 0; i < W5500_SOCK_COUNT; i++) {
            uint8_t ir = w5500_socket_ir(i);
            uint8_t sr = w5500_socket_status(i);

            /* ---- Connection established ---- */
            if (ir & Sn_IR_CON) {
                w5500_socket_ir_clear(i, Sn_IR_CON);
                g_sock[i].connected = 1;
                g_sock[i].last_tick = g_tick_ms;
                static const char *conn[] = {"S0 connected\r\n","S1 connected\r\n","S2 connected\r\n","S3 connected\r\n"};
                socket_connect_num[i] = 1;
                uart_debug(conn[i]);
            }

            /* Fallback: ESTABLISHED detected by status poll */
            if (sr == SOCK_ESTABLISHED && !g_sock[i].connected) {
                g_sock[i].connected = 1;
                g_sock[i].last_tick = g_tick_ms;
                uart_debug("Client connected (poll)\r\n");
            }

            /* ---- Disconnect ---- */
            if (ir & Sn_IR_DISCON) {
                w5500_socket_ir_clear(i, Sn_IR_DISCON);
                g_sock[i].connected = 0;
                w5500_socket_close(i);
                w5500_socket_init(i, W5500_PORT);
                w5500_socket_listen(i);
                sock_inited[i] = 1;
                last_sr[i] = w5500_socket_status(i);
                continue;
            }

            /* ---- Timeout ---- */
            if (ir & Sn_IR_TIMEOUT) {
                w5500_socket_ir_clear(i, Sn_IR_TIMEOUT);
                g_sock[i].connected = 0;
                w5500_socket_close(i);
                w5500_socket_init(i, W5500_PORT);
                w5500_socket_listen(i);
                sock_inited[i] = 1;
                last_sr[i] = w5500_socket_status(i);
                continue;
            }

            /* ---- Data received → Modbus-TCP processing ---- */
            if (ir & Sn_IR_RECV) {
                uint16_t rx_len = w5500_socket_recv_size(i);
                if (rx_len > 0) {
                    uint8_t buf[MB_TCP_RX_BUF_SIZE];
                    uint16_t n = (rx_len < sizeof(buf)) ? rx_len : sizeof(buf);
                    w5500_socket_recv(i, buf, n);
                    modbus_tcp_process(i, buf, n, mb_tcp_send_cb, NULL);
                }
                w5500_socket_ir_clear(i, Sn_IR_RECV);
            }

            /* ---- Silent disconnect detection ---- */
            if (sr != SOCK_ESTABLISHED && g_sock[i].connected) {
                g_sock[i].connected = 0;
                static const char *discon[] = {"S0 disconnected\r\n","S1 disconnected\r\n","S2 disconnected\r\n","S3 disconnected\r\n"};
                socket_connect_num[i] = 0;
                uart_debug(discon[i]);
            }

            /* ---- Recover closed sockets ---- */
            if ((sr == SOCK_CLOSED || sr == SOCK_CLOSE_WAIT) && sock_inited[i]) {
                if (sr != last_sr[i]) {
                    if (g_sock[i].connected) {
                        g_sock[i].connected = 0;
                        w5500_socket_close(i);
                    }
                    w5500_socket_init(i, W5500_PORT);
                    w5500_socket_listen(i);
                    last_sr[i] = w5500_socket_status(i);
                }
            } else if (!sock_inited[i] && sr != 0) {
                last_sr[i] = sr;
                sock_inited[i] = 1;
            }
        }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
