#include "communication.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MASTER_STATE_TRACK        (1U)
#define MASTER_DEFAULT_SPEED      (120.0f)
#define MASTER_DEFAULT_FLAGS      (0U)
#define MASTER_LOOP_PERIOD_US     (20000)

static volatile sig_atomic_t g_master_running = 1;

static void master_signal_handler(int sig)
{
    (void)sig;
    g_master_running = 0;
}

static void master_print_usage(const char *program)
{
    fprintf(stderr, "usage: %s <device> [baudrate] [target_speed]\n", program);
    fprintf(stderr, "example: %s /dev/ttyS4 460800 120\n", program);
}

static void master_setup_signal(void)
{
    signal(SIGINT, master_signal_handler);
    signal(SIGTERM, master_signal_handler);
}

static float master_get_track_error(void)
{
    /*
     * 当前先保留最小巡线接口位置。
     * 后续接入视觉后，这里直接替换成当前图像算法输出的赛道误差即可。
     */
    return 0.0f;
}

static float master_get_target_speed(float default_speed)
{
    /*
     * 当前阶段椭圆巡线先使用固定目标速度。
     * 后续如果需要直道/弯道分速，再在这里接入速度规划逻辑。
     */
    return default_speed;
}

static void master_prepare_track_frame(master_control_frame_t *tx_frame, float default_speed)
{
    tx_frame->track_error  = master_get_track_error();
    tx_frame->target_speed = master_get_target_speed(default_speed);
    tx_frame->state_cmd    = MASTER_STATE_TRACK;
    tx_frame->flags        = MASTER_DEFAULT_FLAGS;
}

int main(int argc, char *argv[])
{
    const char *device;
    int baudrate = 460800;
    float target_speed = MASTER_DEFAULT_SPEED;
    int fd;
    unsigned int loop_count = 0;
    master_comm_parser_t parser = {0};
    master_control_frame_t tx_frame;
    master_feedback_frame_t rx_frame;

    if(argc < 2)
    {
        master_print_usage(argv[0]);
        return 1;
    }

    device = argv[1];
    if(argc >= 3)
    {
        baudrate = atoi(argv[2]);
    }
    if(argc >= 4)
    {
        target_speed = (float)atof(argv[3]);
    }

    fd = master_comm_open(device, baudrate);
    if(fd < 0)
    {
        return 1;
    }

    master_setup_signal();

    printf("serial open success: %s @ %d\n", device, baudrate);
    printf("track mode start, target_speed = %.2f\n", target_speed);
    printf("press Ctrl+C to quit\n");

    memset(&tx_frame, 0, sizeof(tx_frame));
    memset(&rx_frame, 0, sizeof(rx_frame));

    while(g_master_running)
    {
        master_prepare_track_frame(&tx_frame, target_speed);

        if(master_comm_send_control(fd, &tx_frame) != 0)
        {
            break;
        }

        if(master_comm_recv_feedback(fd, &parser, &rx_frame, 20) > 0)
        {
            if(0 == (loop_count % 10U))
            {
                printf("track_error=%7.3f  target=%7.3f  speed=%7.3f  motor=%8d  servo=%5u  state=%3u\n",
                       tx_frame.track_error,
                       tx_frame.target_speed,
                       rx_frame.actual_speed,
                       rx_frame.motor_output,
                       rx_frame.servo_output,
                       rx_frame.state_value);
            }
        }

        loop_count ++;
        usleep(MASTER_LOOP_PERIOD_US);
    }

    master_comm_close(fd);
    return 0;
}
