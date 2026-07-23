#include "ti_msp_dl_config.h"

#include "board_port.h"
#include "car_control.h"

#include <stdint.h>

/*
 * Both wheel speeds are regulated by encoder feedback.
 * Positive values command the configured forward direction.
 */
#define MOTOR_TARGET_SPEED_MPS 0.03f

static void emergency_stop(void)
{
    __disable_irq();
    board_motor_fault_shutdown();
    SYSCFG_DL_motorOutputsLow();
    while (1) {
        __WFI();
    }
}

void NMI_Handler(void)       { emergency_stop(); }
void HardFault_Handler(void) { emergency_stop(); }
void SVC_Handler(void)       { emergency_stop(); }
void PendSV_Handler(void)    { emergency_stop(); }
void SysTick_Handler(void)   { emergency_stop(); }

int main(void)
{
    int32_t encoder_left;
    int32_t encoder_right;

    /*
     * Keep the motor pins low until all clocks, timers and GPIOs are ready.
     * board_init() starts the 100 Hz control timer and enables encoder IRQs.
     */
    SYSCFG_DL_init();
    board_init();
    car_control_init();
    car_control_set_diff_targets(
        MOTOR_TARGET_SPEED_MPS, MOTOR_TARGET_SPEED_MPS);
    board_led_set(true);
    __enable_irq();

    while (1) {
        board_poll();

        while (board_take_control_tick()) {
            board_encoder_snapshot(&encoder_left, &encoder_right);
            car_control_update_encoders(encoder_left, encoder_right);

            /*
             * Refresh the target every control cycle. A latched motor fault
             * still takes priority and prevents the outputs from restarting.
             */
            car_control_set_diff_targets(
                MOTOR_TARGET_SPEED_MPS, MOTOR_TARGET_SPEED_MPS);
            car_control_step();
        }

        __WFI();
    }
}
