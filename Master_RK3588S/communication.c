#include "communication.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

static speed_t master_comm_get_speed(int baudrate)
{
    switch(baudrate)
    {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return 0;
    }
}

static uint8_t master_comm_checksum(const uint8_t *buff, size_t len)
{
    size_t i;
    uint8_t sum = 0;

    for(i = 0; i < len; i ++)
    {
        sum = (uint8_t)(sum + buff[i]);
    }

    return sum;
}

static void master_comm_pack_float(uint8_t *dst, float value)
{
    memcpy(dst, &value, sizeof(float));
}

static void master_comm_pack_int32(uint8_t *dst, int32_t value)
{
    memcpy(dst, &value, sizeof(int32_t));
}

static float master_comm_unpack_float(const uint8_t *src)
{
    float value;
    memcpy(&value, src, sizeof(float));
    return value;
}

static int32_t master_comm_unpack_int32(const uint8_t *src)
{
    int32_t value;
    memcpy(&value, src, sizeof(int32_t));
    return value;
}

static uint16_t master_comm_unpack_uint16(const uint8_t *src)
{
    uint16_t value;
    memcpy(&value, src, sizeof(uint16_t));
    return value;
}

int master_comm_open(const char *device, int baudrate)
{
    int fd;
    struct termios tty;
    speed_t speed;

    speed = master_comm_get_speed(baudrate);
    if(0 == speed)
    {
        fprintf(stderr, "unsupported baudrate: %d\n", baudrate);
        return -1;
    }

    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0)
    {
        perror("open serial device failed");
        return -1;
    }

    if(tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if(tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

void master_comm_close(int fd)
{
    if(fd >= 0)
    {
        close(fd);
    }
}

void master_comm_parser_reset(master_comm_parser_t *parser)
{
    if(NULL != parser)
    {
        parser->rx_index = 0;
    }
}

int master_comm_send_control(int fd, const master_control_frame_t *frame)
{
    uint8_t buff[MASTER_COMM_RX_FRAME_LEN];
    ssize_t write_len;

    if((fd < 0) || (NULL == frame))
    {
        return -1;
    }

    buff[0] = MASTER_COMM_FRAME_HEAD;
    buff[1] = MASTER_COMM_RX_ADDR;
    buff[2] = MASTER_COMM_RX_PAYLOAD_LEN;
    master_comm_pack_float(&buff[3], frame->track_error);
    master_comm_pack_float(&buff[7], frame->target_speed);
    buff[11] = frame->state_cmd;
    buff[12] = frame->flags;
    buff[13] = master_comm_checksum(buff, MASTER_COMM_RX_FRAME_LEN - 1);

    write_len = write(fd, buff, sizeof(buff));
    if(write_len != (ssize_t)sizeof(buff))
    {
        perror("write control frame failed");
        return -1;
    }

    return 0;
}

static int master_comm_try_parse(master_comm_parser_t *parser, master_feedback_frame_t *frame)
{
    uint8_t checksum;

    if(MASTER_COMM_TX_FRAME_LEN != parser->rx_index)
    {
        return 0;
    }

    if(MASTER_COMM_FRAME_HEAD != parser->rx_buffer[0])
    {
        parser->rx_index = 0;
        return 0;
    }

    if(MASTER_COMM_TX_ADDR != parser->rx_buffer[1])
    {
        parser->rx_index = 0;
        return 0;
    }

    if(MASTER_COMM_TX_PAYLOAD_LEN != parser->rx_buffer[2])
    {
        parser->rx_index = 0;
        return 0;
    }

    checksum = master_comm_checksum(parser->rx_buffer, MASTER_COMM_TX_FRAME_LEN - 1);
    if(checksum != parser->rx_buffer[MASTER_COMM_TX_FRAME_LEN - 1])
    {
        parser->rx_index = 0;
        return 0;
    }

    frame->actual_speed = master_comm_unpack_float(&parser->rx_buffer[3]);
    frame->motor_output = master_comm_unpack_int32(&parser->rx_buffer[7]);
    frame->servo_output = master_comm_unpack_uint16(&parser->rx_buffer[11]);
    frame->state_value  = parser->rx_buffer[13];

    parser->rx_index = 0;
    return 1;
}

int master_comm_recv_feedback(int fd, master_comm_parser_t *parser, master_feedback_frame_t *frame, int timeout_ms)
{
    fd_set readfds;
    struct timeval tv;
    uint8_t dat;
    ssize_t read_len;
    int ret;

    if((fd < 0) || (NULL == parser) || (NULL == frame))
    {
        return -1;
    }

    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(fd + 1, &readfds, NULL, NULL, &tv);
    if(ret < 0)
    {
        perror("select failed");
        return -1;
    }

    if(0 == ret)
    {
        return 0;
    }

    while(1)
    {
        read_len = read(fd, &dat, 1);
        if(read_len < 0)
        {
            if((EAGAIN == errno) || (EWOULDBLOCK == errno))
            {
                return 0;
            }
            perror("read feedback failed");
            return -1;
        }

        if(0 == read_len)
        {
            return 0;
        }

        if(0 == parser->rx_index)
        {
            if(MASTER_COMM_FRAME_HEAD != dat)
            {
                continue;
            }
        }

        parser->rx_buffer[parser->rx_index ++] = dat;

        if(parser->rx_index >= MASTER_COMM_TX_FRAME_LEN)
        {
            return master_comm_try_parse(parser, frame);
        }
    }
}
