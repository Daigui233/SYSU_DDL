/*********************************************************************************************************************
* 文件名称          Communication
* 车队名称          中山大学 DDL_大电流
* 开发平台          TC264D
*
* 修改记录
* 日期              作者                备注
* 2026-04-29       Daigui              初版
*********************************************************************************************************************/

#include "Communication.h"

communication_info_struct communication_info;

typedef union
{
    float data_float;
    uint8 data_byte[4];
} comm_float_union;

typedef union
{
    int32 data_int32;
    uint8 data_byte[4];
} comm_int32_union;

typedef union
{
    uint16 data_uint16;
    uint8 data_byte[2];
} comm_uint16_union;

/*********************************************************************************************************************
* 函数名称          communication_checksum
* 功能说明          计算通信校验和
* 参数说明          buff            数据缓冲区
*                   len             参与校验的数据长度
* 返回参数          uint8           累加和低 8 位
* 使用示例          sum = communication_checksum(buff, 13);
* 备注信息          当前阶段采用逐飞旧工程中常见的累加和形式
*********************************************************************************************************************/
static uint8 communication_checksum (const uint8 *buff, uint8 len)
{
    uint8 i;
    uint8 sum = 0;

    for(i = 0; i < len; i ++)
    {
        sum += buff[i];
    }

    return sum;
}

/*********************************************************************************************************************
* 函数名称          communication_init
* 功能说明          通信模块初始化函数
* 参数说明          无
* 返回参数          无
* 使用示例          communication_init();
* 备注信息          当前阶段采用轮询串口方式 后续如需中断接收可继续扩展
*********************************************************************************************************************/
void communication_init (void)
{
    communication_info.rx_index = 0;
    communication_info.rx_ready = 0;

    uart_init(COMM_UART_INDEX, COMM_UART_BAUD, COMM_UART_TX_PIN, COMM_UART_RX_PIN);
}

/*********************************************************************************************************************
* 函数名称          communication_poll
* 功能说明          轮询串口并尝试接收完整控制帧
* 参数说明          无
* 返回参数          无
* 使用示例          communication_poll();
* 备注信息          当前阶段主循环中周期性调用本接口即可
*********************************************************************************************************************/
void communication_poll (void)
{
    uint8 dat;
    control_input_struct input;

    while(uart_query_byte(COMM_UART_INDEX, &dat))
    {
        communication_rx_byte(dat);
    }

    if(communication_info.rx_ready)
    {
        if(communication_decode_frame(&input))
        {
            control_set_input(input);
        }

        communication_info.rx_ready = 0;
    }
}

/*********************************************************************************************************************
* 函数名称          communication_rx_byte
* 功能说明          按字节接收控制帧
* 参数说明          dat             接收到的单字节数据
* 返回参数          无
* 使用示例          communication_rx_byte(dat);
* 备注信息          当前阶段使用固定帧长解析
*********************************************************************************************************************/
void communication_rx_byte (uint8 dat)
{
    if(0 == communication_info.rx_index)
    {
        if(COMM_FRAME_HEAD != dat)
        {
            return;
        }
    }

    communication_info.rx_buffer[communication_info.rx_index ++] = dat;

    if(COMM_RX_FRAME_LEN <= communication_info.rx_index)
    {
        communication_info.rx_index = 0;
        communication_info.rx_ready = 1;
    }
}

/*********************************************************************************************************************
* 函数名称          communication_decode_frame
* 功能说明          解码接收到的控制帧
* 参数说明          input           输出的控制输入结构体
* 返回参数          uint8           1 表示解码成功 0 表示失败
* 使用示例          if(communication_decode_frame(&input))
* 备注信息          当前协议内容为 track_error target_speed state_cmd flags
*********************************************************************************************************************/
uint8 communication_decode_frame (control_input_struct *input)
{
    comm_float_union track_error;
    comm_float_union target_speed;
    uint8 checksum;
    uint8 i;

    if(COMM_FRAME_HEAD != communication_info.rx_buffer[0])
    {
        return 0;
    }

    if(COMM_RX_ADDR != communication_info.rx_buffer[1])
    {
        return 0;
    }

    if(COMM_RX_PAYLOAD_LEN != communication_info.rx_buffer[2])
    {
        return 0;
    }

    checksum = communication_checksum(communication_info.rx_buffer, COMM_RX_FRAME_LEN - 1);
    if(checksum != communication_info.rx_buffer[COMM_RX_FRAME_LEN - 1])
    {
        return 0;
    }

    for(i = 0; i < 4; i ++)
    {
        track_error.data_byte[i] = communication_info.rx_buffer[3 + i];
        target_speed.data_byte[i] = communication_info.rx_buffer[7 + i];
    }

    input->track_error = track_error.data_float;
    input->target_speed = target_speed.data_float;
    input->state_cmd = communication_info.rx_buffer[11];
    input->flags = communication_info.rx_buffer[12];

    return 1;
}

/*********************************************************************************************************************
* 函数名称          communication_send_feedback
* 功能说明          打包并发送下位机反馈数据
* 参数说明          无
* 返回参数          无
* 使用示例          communication_send_feedback();
* 备注信息          当前阶段回传实际速度 电机输出 舵机输出 当前状态
*********************************************************************************************************************/
void communication_send_feedback (void)
{
    control_ctx_struct *ctx;
    comm_float_union actual_speed;
    comm_int32_union motor_output;
    comm_uint16_union servo_output;
    uint8 i;

    ctx = control_get_ctx();

    actual_speed.data_float = ctx->actual_speed;
    motor_output.data_int32 = ctx->motor_output;
    servo_output.data_uint16 = (uint16)ctx->servo_output;

    communication_info.tx_buffer[0] = COMM_FRAME_HEAD;
    communication_info.tx_buffer[1] = COMM_TX_ADDR;
    communication_info.tx_buffer[2] = COMM_TX_PAYLOAD_LEN;

    for(i = 0; i < 4; i ++)
    {
        communication_info.tx_buffer[3 + i] = actual_speed.data_byte[i];
        communication_info.tx_buffer[7 + i] = motor_output.data_byte[i];
    }

    communication_info.tx_buffer[11] = servo_output.data_byte[0];
    communication_info.tx_buffer[12] = servo_output.data_byte[1];
    communication_info.tx_buffer[13] = (uint8)state_get();
    communication_info.tx_buffer[14] = communication_checksum(communication_info.tx_buffer, COMM_TX_FRAME_LEN - 1);

    uart_write_buffer(COMM_UART_INDEX, communication_info.tx_buffer, COMM_TX_FRAME_LEN);
}

