#include "main.h"
#include "stm32f10x.h"
#include "debug/bsp_debug.h"
#include "i2c/bsp_i2c.h" 
#include "dwt/bsp_dwt.h" 
#include "oled/bsp_i2c_oled.h"
#include "fonts/bsp_fonts.h"
#include "key/bsp_gpio_key.h"
#include "led/bsp_gpio_led.h"
#include "dht11/bsp_dht11.h"
#include "systick/bsp_systick.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    KEY_Init();
    LED_GPIO_Config();
    IIC_Init();
    DEBUG_USART_Init();
    DHT11_GPIO_Config();
    
    SysTick_Init();
    DWT_Init();
    
    OLED_Init();
    OLED_CLS();
    Dht11_TaskInit(500);
    
    Boot_Task();
    
    menu_show_flag = 1;
    
    while(1)
    {
        /* OLED显示屏任务 */
        Menu_Task();
        
        /* DHT11温湿度传感器任务 */
        Dht11_Task();
    }
        
}

