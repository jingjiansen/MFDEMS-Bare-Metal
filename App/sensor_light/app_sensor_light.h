#ifndef __APP_SENSOR_LIGHT_H
#define __APP_SENSOR_LIGHT_H

#include "stm32f10x.h"//或#include "stdint.h"

typedef struct
{
    uint32_t cycle;
    uint32_t timer;
    uint8_t  flag;
    char     device_id[10];
}SL_TaskInfo;

extern SL_TaskInfo sl_task;

/* 感应灯模式 */
typedef enum 
{
    SL_MODE_AUTO = 0, 
    SL_MODE_OPEN = 1,
    SL_MODE_SHUT = 2,
    SL_MODE_NUM  =3,    //使用此符号的值刚好可以作为指示以上的枚举个数  
}SL_ModeTpye;

/* 反射传感器触发状态 */
typedef enum 
{
    SL_SWITCH_SIGNAL_NONE = 0, 
    SL_SWITCH_SIGNAL_TRIGGER = 1, 
}SL_SwitchSignalTpye;

/* 感应灯信息结构体 */
typedef struct
{
    SL_ModeTpye         mode;
    SL_SwitchSignalTpye last_signal;
    uint64_t            first_trigger_time;
}SL_StateInfo;

#define SL_SWITCH_HOLD_MS 1200 //反射传感器触发保持时间（MS）


void SL_TaskReset(void);
void SL_TaskInit(uint32_t sl_task_cycle);
void SL_Task(void);
void SL_SetMode(SL_ModeTpye sl_mode);
void SL_SwitchMode(void);
void SL_Rd03Handle(void);
void SL_TrackeHandle(void);
#endif /* __APP_SENSOR_LIGHT_H */
