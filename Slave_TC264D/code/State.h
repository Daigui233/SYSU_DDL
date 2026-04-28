/*********************************************************************************************************************
* 文件名称          State
* 车队名称          中山大学 DDL_大电流
* 开发平台          TC264D
*
* 修改记录
* 日期              作者                备注
* 2026-04-29       Daigui              初版
*********************************************************************************************************************/

#ifndef CODE_STATE_H_
#define CODE_STATE_H_

#include "zf_common_headfile.h"

/*********************************************************************************************************************
*                                               下位机状态枚举
*********************************************************************************************************************/
typedef enum
{
    STATE_IDLE = 0,
    STATE_TRACK,
    STATE_LIMIT_SPEED,
    STATE_WAIT_LIGHT,
    STATE_AVOID,
    STATE_NAV_LEFT,
    STATE_NAV_RIGHT,
    STATE_SAFE_STOP,
    STATE_MAX
} car_state_enum;

typedef struct
{
    car_state_enum current_state;
    car_state_enum last_state;
    uint8 state_changed;
} state_info_struct;

extern state_info_struct state_info;

void           state_init             (void);
void           state_set              (car_state_enum state);
car_state_enum state_get              (void);
uint8          state_is_changed       (void);
void           state_clear_changed    (void);

#endif
