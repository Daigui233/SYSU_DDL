/*********************************************************************************************************************
 * �ļ�����          Communication
 * ��������          ��ɽ��ѧ DDL_�����
 * ����ƽ̨          TC264D
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2026-04-29       Daigui              ����
 *********************************************************************************************************************/

#ifndef CODE_COMMUNICATION_H_
#define CODE_COMMUNICATION_H_

#include "zf_common_headfile.h"
#include "Control.h"
#include "State.h"
#include "zf_driver_uart.h"

/* ��ǰ�׶�ͨ�Ŵ��ڲ������ϴ��������а弶����1�������� */
#define COMM_UART_INDEX (UART_1)
#define COMM_UART_BAUD (460800)
#define COMM_UART_TX_PIN (UART1_TX_P20_10)
#define COMM_UART_RX_PIN (UART1_RX_P33_13)

#define COMM_FRAME_HEAD (0x42)
#define COMM_RX_ADDR (0x10)
#define COMM_TX_ADDR (0x90)

#define COMM_RX_PAYLOAD_LEN (10) //����10����������Ӧʮ������ 0x0A
#define COMM_RX_FRAME_LEN (14)
#define COMM_TX_PAYLOAD_LEN (46)
#define COMM_TX_FRAME_LEN (50)

typedef struct
{
    uint8 rx_buffer[COMM_RX_FRAME_LEN];
    uint8 rx_index;
    uint8 rx_ready;

    uint8 tx_buffer[COMM_TX_FRAME_LEN];
} communication_info_struct;

extern communication_info_struct communication_info;

// ��ǰ���������ṹ�壬��Ȼ���뱨��
struct control_input_struct;

// ͨ����ʱ����
extern uint8 communicate_temp;

void communication_init(void);
void communication_poll(void);
void communication_rx_byte(uint8 dat);
uint8 communication_decode_frame(struct control_input_struct *input);
void communication_send_feedback(void);
void communication_itrpt_init(void);

#endif
