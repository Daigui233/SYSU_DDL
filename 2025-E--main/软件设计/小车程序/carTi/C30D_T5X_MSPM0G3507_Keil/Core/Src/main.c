#include "ti_msp_dl_config.h"

#include "app.h"
#include "board_port.h"

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
    /*
     * Generated hardware initialization keeps every motor input low first.
     * app_init() then initializes the board, speed loop, line following and
     * command transports. The application starts in STOP mode.
     */
    SYSCFG_DL_init();
    app_init();
    __enable_irq();

    while (1) {
        app_process();
        __WFI();
    }
}
