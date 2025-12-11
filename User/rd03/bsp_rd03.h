#ifndef __BSP_RD03_H
#define __BSP_RD03_H

#include "stm32f10x.h"
#include <stdio.h>

#define RD03_USART_NUM 3

#if (RD03_USART_NUM == 2)

    #define RD03_TX_GPIO_PORT    			        GPIOA			                /* 对应GPIO端口 */
    #define RD03_TX_GPIO_CLK_PORT 	                RCC_APB2Periph_GPIOA			/* 对应GPIO端口时钟位 */
    #define RD03_TX_GPIO_PIN			            GPIO_Pin_2	       				/* 对应PIN脚 */
    #define RD03_RX_GPIO_PORT    			        GPIOA			                /* 对应GPIO端口 */
    #define RD03_RX_GPIO_CLK_PORT 	                RCC_APB2Periph_GPIOA			/* 对应GPIO端口时钟位 */
    #define RD03_RX_GPIO_PIN			            GPIO_Pin_3	       				/* 对应PIN脚 */
                                                                                
    #define RD03_USARTX   			                USART2                          /* 对应串口号 */
    #define RD03_USARTX_CLK_PORT 	                RCC_APB1Periph_USART2			/* 对应串口外设时钟位 */
    #define RD03_APBXCLKCMD   			            RCC_APB1PeriphClockCmd	        /* 对应串口外设时钟 */
    #define RD03_BAUDRATE   			            115200                          /* 波特率 */
    
    #define RD03_IRQ                                USART2_IRQn                      /* 对应串口中断号 */
    #define RD03_IRQHANDLER                         USART2_IRQHandler                /* 对应串口中断处理函数 */
    
#elif (RD03_USART_NUM == 3)

    #define RD03_TX_GPIO_PORT    			        GPIOB			                /* 对应GPIO端口 */
    #define RD03_TX_GPIO_CLK_PORT 	                RCC_APB2Periph_GPIOB			/* 对应GPIO端口时钟位 */
    #define RD03_TX_GPIO_PIN			            GPIO_Pin_10	       				/* 对应PIN脚 */
    #define RD03_RX_GPIO_PORT    			        GPIOB			                /* 对应GPIO端口 */
    #define RD03_RX_GPIO_CLK_PORT 	                RCC_APB2Periph_GPIOB			/* 对应GPIO端口时钟位 */
    #define RD03_RX_GPIO_PIN			            GPIO_Pin_11	       				/* 对应PIN脚 */
                                                                                
    #define RD03_USARTX   			                USART3                          /* 对应串口号 */
    #define RD03_USARTX_CLK_PORT 	                RCC_APB1Periph_USART3			/* 对应串口外设时钟位 */
    #define RD03_APBXCLKCMD   			            RCC_APB1PeriphClockCmd	        /* 对应串口外设时钟 */
    #define RD03_BAUDRATE   			            115200		                    /* 波特率 */
    
    #define RD03_IRQ                                USART3_IRQn                      /* 对应串口中断号 */
    #define RD03_IRQHANDLER                         USART3_IRQHandler                /* 对应串口中断处理函数 */
    
#endif

#define RD03_BUFFER_SIZE 1024 

/* RD03串口数据结构体 */
typedef struct
{
    uint8_t    buffer[RD03_BUFFER_SIZE];
    uint32_t   len;
    uint32_t   read_flag;  
}RD03_DataTypeDef;

extern RD03_DataTypeDef rd03_receive;

void RD03_NVIC_Config(void);
void RD03_USART_PinConfig(void);
void RD03_USART_ModeConfig(void);
void RD03_USART_Init(void);

#endif /* __BSP_RD03_H  */
