/*********************************************************************************************************************
* 文件名称          Init
* 车队名称          中山大学 DDL_大电流
* 开发平台          TC264D
*
* 修改记录
* 日期              作者                备注
* 2026-04-28       Daigui              初版
* 2026-04-29       Daigui              接入 State 与 Control 模块初始化
* 2026-04-29       Daigui              接入 Communication 模块初始化
*********************************************************************************************************************/

#ifndef CODE_INIT_H_
#define CODE_INIT_H_

#include "zf_common_headfile.h"

#include "Motor.h"
#include "Servo.h"
#include "State.h"
#include "Control.h"
#include "Communication.h"

void total_init             (void);

#endif
