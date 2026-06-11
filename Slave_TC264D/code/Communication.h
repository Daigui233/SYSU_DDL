/*********************************************************************************************************************
 * 文件名称          Communication
 * 文件说明          SYSU_DDL 下位机通信模块
 * 适用平台          TC264D
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-04-29       Daigui              创建
 * 2026-06-03       Codex               扩展反馈帧并保持 GBK 编码
 *********************************************************************************************************************/

#ifndef CODE_COMMUNICATION_H_
#define CODE_COMMUNICATION_H_

#include "zf_common_headfile.h"
#include "Control.h"
#include "State.h"
#include "zf_driver_uart.h"

/* 当前控车串口使用 UART_1，波特率 460800。 */
#define COMM_UART_INDEX (UART_1)
#define COMM_UART_BAUD (460800)
#define COMM_UART_TX_PIN (UART1_TX_P20_10)
#define COMM_UART_RX_PIN (UART1_RX_P33_13)

#define COMM_FRAME_HEAD (0x42)
#define COMM_RX_ADDR (0x10)
#define COMM_TX_ADDR (0x90)

#define COMM_RX_PAYLOAD_LEN (10) // 控制帧数据区长度
#define COMM_RX_FRAME_LEN (14)
#define COMM_TX_PAYLOAD_LEN (70)
#define COMM_TX_FRAME_LEN (74)

typedef struct
{
    uint8 rx_buffer[COMM_RX_FRAME_LEN];
    uint8 rx_index;
    uint8 rx_ready;

    uint8 tx_buffer[COMM_TX_FRAME_LEN];
} communication_info_struct;

extern communication_info_struct communication_info;

// 前向声明，避免循环包含。
struct control_input_struct;

// 通信临时接收字节。
extern uint8 communicate_temp;

void communication_init(void);
void communication_poll(void);
void communication_rx_byte(uint8 dat);
uint8 communication_decode_frame(struct control_input_struct *input);
void communication_send_feedback(void);
void communication_itrpt_init(void);

#endif