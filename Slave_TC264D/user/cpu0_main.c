#include "zf_common_headfile.h"
#include "Init.h"
#pragma section all "cpu0_dsram"

// ===== ENCODER_SIGN_TEST_START: lift wheels, set ENABLE=1, positive duty should spin forward; feedback actual_speed is raw encoder count, >0 keeps MOTOR_ENCODER_SIGN=1.0f, <0 means use -1.0f; delete this block after test. =====
#define ENCODER_SIGN_TEST_ENABLE (0)
#define ENCODER_SIGN_TEST_DUTY   (1500)
// ===== ENCODER_SIGN_TEST_END =====


int core0_main(void)
{
    clock_init();
    debug_init();

    cpu_wait_event_ready();

    total_init();
    control_ctx.input.target_speed = 0.0f;

    while (TRUE)
    {
        while (control_ctx.periodic_interrupt_flag == 1)
        {
            control_ctx.periodic_interrupt_flag = 0;
#if ENCODER_SIGN_TEST_ENABLE
            state_set(STATE_TRACK);
            motor_set_duty(ENCODER_SIGN_TEST_DUTY);
            control_ctx.actual_speed = (float)motor_get_speed();
            control_ctx.motor_target = (float)ENCODER_SIGN_TEST_DUTY;
            control_ctx.motor_output = ENCODER_SIGN_TEST_DUTY;
            control_ctx.input.target_speed = 0.0f;
            control_ctx.input.track_error = 0.0f;
            control_ctx.input.state_cmd = (uint8)STATE_TRACK;
            control_ctx.input.flags = 0;
            communication_send_feedback();
#else
            control_update();
            communication_send_feedback();
#endif
        }
    }
}

#pragma section all restore
