
#include "Communication.h"

communication_info_struct communication_info;

uint8 communicate_temp;

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

static void communication_put_float(uint8 index, float value)
{
    comm_float_union data;
    uint8 i;

    data.data_float = value;
    for (i = 0; i < 4; i++)
    {
        communication_info.tx_buffer[index + i] = data.data_byte[i];
    }
}

static void communication_put_int32(uint8 index, int32 value)
{
    comm_int32_union data;
    uint8 i;

    data.data_int32 = value;
    for (i = 0; i < 4; i++)
    {
        communication_info.tx_buffer[index + i] = data.data_byte[i];
    }
}
static uint8 communication_checksum(const uint8 *buff, uint8 len)
{
    uint8 i;
    uint8 sum = 0;

    for (i = 0; i < len; i++)
    {
        sum += buff[i];
    }

    return sum;
}

void communication_init(void)
{
    communication_info.rx_index = 0;
    communication_info.rx_ready = 0;

    uart_init(COMM_UART_INDEX, COMM_UART_BAUD, COMM_UART_TX_PIN, COMM_UART_RX_PIN);
}

void communication_poll(void)
{
    uint8 dat;
    control_input_struct input;

    while (uart_query_byte(COMM_UART_INDEX, &dat))
    {
        communication_rx_byte(dat);
    }

    if (communication_info.rx_ready)
    {
        if (communication_decode_frame(&input))
        {
            control_set_input(input);
        }

        communication_info.rx_ready = 0;
    }
}

void communication_itrpt_init(void)
{
    communication_info.rx_index = 0;
    communication_info.rx_ready = 0;

    uart_init(COMM_UART_INDEX, COMM_UART_BAUD, COMM_UART_TX_PIN, COMM_UART_RX_PIN);

    uart_rx_interrupt(COMM_UART_INDEX, 1);
}

void communication_rx_byte(uint8 dat)
{
    if (0 == communication_info.rx_index)
    {
        if (COMM_FRAME_HEAD != dat)
        {
            return;
        }
    }

    communication_info.rx_buffer[communication_info.rx_index++] = dat;

    if (COMM_RX_FRAME_LEN <= communication_info.rx_index)
    {
        communication_info.rx_index = 0;
        communication_info.rx_ready = 1;
    }
}

uint8 communication_decode_frame(struct control_input_struct *input)
{
    comm_float_union track_error;
    comm_float_union target_speed;
    uint8 checksum;
    uint8 i;

    if (COMM_FRAME_HEAD != communication_info.rx_buffer[0])
    {
        return 0;
    }

    if (COMM_RX_ADDR != communication_info.rx_buffer[1])
    {
        return 0;
    }

    if (COMM_RX_PAYLOAD_LEN != communication_info.rx_buffer[2])
    {
        return 0;
    }

    checksum = communication_checksum(communication_info.rx_buffer, COMM_RX_FRAME_LEN - 1);
    if (checksum != communication_info.rx_buffer[COMM_RX_FRAME_LEN - 1])
    {
        return 0;
    }

    for (i = 0; i < 4; i++)
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

void communication_send_feedback(void)
{
    control_ctx_struct *ctx;

    ctx = control_get_ctx();

    communication_info.tx_buffer[0] = COMM_FRAME_HEAD;
    communication_info.tx_buffer[1] = COMM_TX_ADDR;
    communication_info.tx_buffer[2] = COMM_TX_PAYLOAD_LEN;

    communication_put_float(3, ctx->actual_speed);
    communication_put_float(7, ctx->motor_target);
    communication_put_float(11, ctx->input.target_speed);
    communication_put_float(15, ctx->input.track_error);
    communication_put_int32(19, ctx->motor_output);
    communication_put_int32(23, (int32)ctx->servo_output);
    communication_put_float(27, ctx->param.motor_kp);
    communication_put_float(31, ctx->param.motor_ki);
    communication_put_float(35, ctx->param.motor_kd);
    communication_put_float(39, ctx->param.servo_kp);
    communication_put_float(43, ctx->param.servo_kd);
    communication_info.tx_buffer[47] = (uint8)state_get();
    communication_info.tx_buffer[48] = ctx->input.flags;
    communication_info.tx_buffer[49] = communication_checksum(communication_info.tx_buffer, COMM_TX_FRAME_LEN - 1);

    uart_write_buffer(COMM_UART_INDEX, communication_info.tx_buffer, COMM_TX_FRAME_LEN);
}
