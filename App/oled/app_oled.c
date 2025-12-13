#include "oled/app_oled.h" 
#include "oled/bsp_i2c_oled.h"
#include "fonts/bsp_fonts.h"
#include "dwt/bsp_dwt.h" 
#include "key/bsp_gpio_key.h"
#include "led/bsp_gpio_led.h"
#include "i2c/bsp_i2c.h"
#include "dht11/app_dht11.h"
#include <stdio.h>

uint8_t menu = 0;
uint8_t menu_show_flag = 0;
uint8_t content_show_flag = 0;
uint8_t left_shift_flag = 0;
uint8_t right_shift_flag = 0;
uint8_t led_state_flag = 0;
uint32_t wooden_fish_num = 0;


/**
 * @brief  开机悬停界面任务
 * @param  无
 * @retval 无
 */
void Boot_Task(void)
{
    OLED_DrawBitMap((128-113)/2, 0, 113, 64, (uint8_t*)dog4);
    DWT_DelayMs(200);
    OLED_DrawBitMap((128-113)/2, 0, 113, 64, (uint8_t*)dog3);
    DWT_DelayMs(300);
    OLED_DrawBitMap((128-113)/2, 0, 113, 64, (uint8_t*)dog1);
    DWT_DelayMs(500);

    //    OLED_DrawBitMap((128-113)/2, 0, 113, 64, (uint8_t*)dog4);
    //    DWT_DelayMs(1000);
    // /* 显示野火电子--上下左右居中 */
    // OLED_ShowChinese_F16X16(1,(8-4)/2+0,0);
    // OLED_ShowChinese_F16X16(1,(8-4)/2+1,1);
    // OLED_ShowChinese_F16X16(1,(8-4)/2+2,2);
    // OLED_ShowChinese_F16X16(1,(8-4)/2+3,3);
    // IIC_DELAY_US(1000000);
}

/**
 * @brief  OLED任务
 * @param  无
 * @retval 无
 */
void Menu_Task(void)
{
    /*按键3按下判断是否切换到内容界面*/
    if(KEY_Scan(KEY3_GPIO_PORT, KEY3_GPIO_PIN,KEY_HIGH_TRIGGER) == KEY_DOWN)
    {
        /*菜单选择：低4位为0时，一级菜单进入二级菜单显示内容*/
        if((menu&0x0f)==0)
        {
            OLED_CLS(); 
            content_show_flag = 1;
            if(menu == 0x00) /*菜单1进入还是菜单1*/
            {
                menu = menu;
            }
            else if(menu == 0x30) /*菜单3进入则显示灯具内容*/
            {
                /* 灯具内容：开关灯，依据led_state_flag判断当前状态切换显示的位图*/
                if(led_state_flag == 1)
                {
                    menu = (menu&0xf0)|0x02; /*显示关灯位图*/
                }
                else
                {
                    menu = (menu&0xf0)|0x01; /*显示开灯位图*/
                }
            }
            else /*其他菜单进入则显示菜单内容*/
            {
                menu = (menu&0xf0)|0x01;
            }
        }
        /* 菜单选择：低4位不为0时，退回一级菜单 */
        else
        {
            OLED_CLS(); 
            DWT_DelayMs(200);
    
            menu_show_flag = 1;
            menu = menu&0xf0;
        }
    }
    /*按键1和按键2按下判断是否切换*/
    if(left_shift_flag == 1||right_shift_flag == 1)
    {
        /*通过低4位判断是否显示的是内容还是主页，为0表示显示主页*/
        if((menu&0x0f)==0)
        {
            if(left_shift_flag)
            {
                if(menu == 0x00) /*第一菜单再向左切换，切换到第四菜单*/
                {
                    menu = 0x40;
                }
                else
                {
                    menu = menu-0x10; /*其他菜单向左切换，切换到上一级菜单*/
                
                }
            }
            if(right_shift_flag) /*第四菜单再向右切换，切换到第一菜单*/
            {
                if(menu == 0x40)
                {
                    menu = 0x00;
                }
                else
                {
                    menu = menu+0x10; /*其他菜单向右切换，切换到下一级菜单*/
                
                }
            }
            right_shift_flag = 0;
            left_shift_flag = 0;
            menu_show_flag = 1;
        }
        /*显示内容*/
        else
        {
            /*内容页左键按下*/
            if(left_shift_flag)
            {
                if(menu == 0x31) /*如果是关灯，则切换到开灯*/
                {
                    menu = 0x32;
                }
                else if(menu == 0x32) /*如果是开灯，则切换到关灯*/
                {
                    menu = 0x31;
                }
                if(menu == 0x41) /*如果是打坐，则增加打坐次数*/
                {
                    wooden_fish_num++;
                }
            }
            /*内容页右键按下*/
            if(right_shift_flag) 
            {
                if(menu == 0x31) /*如果是开灯，则切换到关灯*/
                {
                    menu = 0x32;
                }
                else if(menu == 0x32) /*如果是关灯，则切换到开灯*/
                {
                    menu = 0x31;
                }
                if(menu == 0x41) /*如果是打坐，则清零*/
                {
                    wooden_fish_num = 0;
                }
            }
            right_shift_flag = 0; /*右键释放*/
            left_shift_flag = 0; /*左键释放*/
            content_show_flag = 1; /*内容显示标志清空*/
        }
    }

    /*是否需要显示主页*/
    if((menu&0x0f) == 0 && menu_show_flag == 1)
    {
        OLED_CLS(); 
        switch(menu)
        { 
            case 0x00://显示主页
                OLED_DrawBitMap((128-50)/2-1,1,50,50,(uint8_t*)home);
                break;
            case 0x10://显示音乐
                OLED_DrawBitMap((128-50)/2-1,1,50,50,(uint8_t*)music);
                OLED_ShowChinese_F16X16(2,0,11);
                OLED_ShowChinese_F16X16(2,7,12); 
                break;
            case 0x20://显示温湿度
                OLED_DrawBitMap((128-50)/2,1,50,50,(uint8_t*)humidity);
                OLED_ShowChinese_F16X16(2,0,11);
                OLED_ShowChinese_F16X16(2,7,12); 
                break;
            case 0x30://控制灯具
                OLED_DrawBitMap((128-50)/2-1,1,50,50,(uint8_t*)lighting_control);
                OLED_ShowChinese_F16X16(2,0,11);
                OLED_ShowChinese_F16X16(2,7,12); 
                break;
            case 0x40://打坐
                OLED_DrawBitMap((128-50)/2-1,1,50,50,(uint8_t*)meditate);
                OLED_ShowChinese_F16X16(2,0,11);
                OLED_ShowChinese_F16X16(2,7,12);             
            default:
                break;
        }
        menu_show_flag = 0;
    }
    /*显示内容*/
    else
    {
        if(content_show_flag == 1)
        {
            switch(menu&0xf0)
            { 
                case 0x00://显示主页
                    OLED_DrawBitMap((128-50)/2-1,1,50,50,(uint8_t*)home);
                    break;
                case 0x10://显示音乐
                    if(menu ==0x11)
                    {
                        OLED_DrawBitMap(0,0,128,64,(uint8_t*)music1);
                    }
                    break;
                case 0x20://显示温湿度
                    if(menu == 0x21)//第一次进来
                    {
                        OLED_CLS();
                        
                        OLED_ShowChinese_F16X16(1,1,6);
                        OLED_ShowChinese_F16X16(1,2,8);
                        OLED_ShowChinese_F16X16(1,3,13);
                    
                        OLED_ShowChinese_F16X16(3,1,7);
                        OLED_ShowChinese_F16X16(3,2,8);
                        OLED_ShowChinese_F16X16(3,3,13);
                        
                        menu =0x22;
                    }
                    if(menu == 0x22)
                    {
                        char str_temp[512] = {NULL};
                        char str_temp1[512] = {NULL};
                        OLED_SetPos(1,64);
                        OLED_WriteBuffer(OLED_WR_DATA,(uint8_t*)str_temp,128-64);
                        OLED_SetPos(3,64);
                        OLED_WriteBuffer(OLED_WR_DATA,(uint8_t*)str_temp1,128-64);
                                                     
                        OLED_ShowChinese_F16X16(1,7,14);
                        if(dht11_data.temp_deci&0x80)//判断是否低于0
                        {
                            sprintf(str_temp,"-%d.%d",dht11_data.temp_int,dht11_data.temp_deci);
                        
                        }
                        else
                        {
                            sprintf(str_temp,"%d.%d",dht11_data.temp_int,dht11_data.temp_deci);
                        }
                        if(dht11_data.humi_deci&0x80)//判断是否低于0
                        {
                            sprintf(str_temp1,"-%d.%d %%RH",dht11_data.humi_int,dht11_data.humi_deci);
                        
                        }
                        else
                        {
                            sprintf(str_temp1,"%d.%d %%RH",dht11_data.humi_int,dht11_data.humi_deci);
                        
                        }
                        OLED_ShowString_F8X16(1,8,(uint8_t*)str_temp);
                        OLED_ShowString_F8X16(3,8,(uint8_t*)str_temp1);
                        
                    }
                    break;
                case 0x30://控制灯具
                    if(menu == 0x31)
                    {
                        OLED_DrawBitMap((128-52)/2-1,(64-20)/2,52,20,(uint8_t*)led_off_bmp);
                        led_state_flag = 0;
                        LED_OFF(LED4_GPIO_PORT,LED4_GPIO_PIN,LED_LOW_TRIGGER);
                    }
                    else if(menu == 0x32)
                    {
                        OLED_DrawBitMap((128-52)/2-1,(64-20)/2,52,20 ,(uint8_t*)led_on_bmp);
                        led_state_flag = 1;
                        LED_ON(LED4_GPIO_PORT,LED4_GPIO_PIN,LED_LOW_TRIGGER);
                    }                       
                    break;
                case 0x40://打坐
                    if(menu == 0x41)
                    {
                        OLED_DrawBitMap((128-85)/2-1,64-50,85,50,(uint8_t*)wooden_fish_on);
                        IIC_DELAY_US(200000);
                        OLED_CLS();
                        OLED_DrawBitMap((128-67)/2-1,64-50,67,50,(uint8_t*)wooden_fish_off);
                    }
                    {
                        char str_temp1[128] = {NULL};
                        sprintf(str_temp1, "%d",wooden_fish_num);
                        OLED_SetPos(0,64);
                        OLED_ShowString_F8X16(0,7,(uint8_t*)str_temp1);
                    }
                    break;
                default:
                    break;
            }
            content_show_flag = 0;
        }
    }
}

/*****************************END OF FILE***************************************/

