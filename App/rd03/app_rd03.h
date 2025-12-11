#ifndef __APP_RD03_H
#define __APP_RD03_H

#include "stm32f10x.h"//或#include "stdint.h"

/* RD03雷达区域目标搜索状态 */
typedef enum 
{
    RD03_NOEXIST = 0, 
    RD03_EXIST = 1,
}Rd03_ExistTpye;

/* RD03雷达区域目标信息结构体 */
typedef struct
{
    Rd03_ExistTpye exist;
    uint32_t       range;
}Rd03_ResultInfo;

void Rd03_Running_Handle(uint8_t* buffer,uint32_t size);
void Rd03_ReadBufferReset(void);
void Rd03_TaskInit(void);
void Rd03_Task(void);
Rd03_ResultInfo Rd03_GetResult(void);

#endif /* __APP_RD03_H */
