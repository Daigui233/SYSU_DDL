/*********************************************************************************************************************
 * 文件名称          Init
 * 文件说明          SYSU_DDL 下位机初始化模块
 * 适用平台          TC264D
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-04-28       Daigui              创建
 * 2026-04-29       Daigui              接入 State 和 Control 初始化
 * 2026-04-29       Daigui              接入 Communication 初始化
 * 2026-06-03       Codex               调整控制周期并保持 GBK 编码
 *********************************************************************************************************************/

#include "Init.h"

/*********************************************************************************************************************
 * 函数名称          total_init
 * 函数说明          下位机总初始化入口
 * 参数说明          无
 * 返回参数          无
 * 使用示例          total_init();
 * 备注信息          初始化电机、舵机、状态、控制周期和通信中断。
 *********************************************************************************************************************/
void total_init(void)
{
    gpio_init(P20_8, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P20_9, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P21_5, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P21_4, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    motor_init();
    servo_init();
    state_init();
    control_init();
    pit_set_and_enable(10);
    // communication_init();
    communication_itrpt_init();
}