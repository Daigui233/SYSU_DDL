#include "zf_common_headfile.h"
#include "Init.h"
#pragma section all "cpu0_dsram"

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
            control_update();
            communication_send_feedback();
        }
    }
}

#pragma section all restore
