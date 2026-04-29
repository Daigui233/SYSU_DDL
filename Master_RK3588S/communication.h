#ifndef MASTER_COMMUNICATION_H_
#define MASTER_COMMUNICATION_H_

#include <stdint.h>
#include <stddef.h>

#define MASTER_COMM_FRAME_HEAD       (0x42)
#define MASTER_COMM_RX_ADDR          (0x10)
#define MASTER_COMM_TX_ADDR          (0x90)
#define MASTER_COMM_RX_PAYLOAD_LEN   (10)
#define MASTER_COMM_RX_FRAME_LEN     (14)
#define MASTER_COMM_TX_PAYLOAD_LEN   (11)
#define MASTER_COMM_TX_FRAME_LEN     (15)

typedef struct
{
    float   track_error;
    float   target_speed;
    uint8_t state_cmd;
    uint8_t flags;
} master_control_frame_t;

typedef struct
{
    float   actual_speed;
    int32_t motor_output;
    uint16_t servo_output;
    uint8_t state_value;
} master_feedback_frame_t;

typedef struct
{
    uint8_t rx_buffer[MASTER_COMM_TX_FRAME_LEN];
    size_t  rx_index;
} master_comm_parser_t;

int master_comm_open(const char *device, int baudrate);
void master_comm_close(int fd);
int master_comm_send_control(int fd, const master_control_frame_t *frame);
int master_comm_recv_feedback(int fd, master_comm_parser_t *parser, master_feedback_frame_t *frame, int timeout_ms);
void master_comm_parser_reset(master_comm_parser_t *parser);

#endif
