#include<ESC.h>
/******************************************************************************
      函数说明：初始化电调
      入口数据：
               
      返回值：  无
******************************************************************************/
void ESC_init(void){
    printf("\nStart\n");
    HAL_TIM_Base_Start_IT(&ESC_PWM_LINE);
    HAL_TIM_PWM_Start(&ESC_PWM_LINE,TIM_CHANNEL_1);    
    HAL_TIM_PWM_Start(&ESC_PWM_LINE,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&ESC_PWM_LINE,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&ESC_PWM_LINE,TIM_CHANNEL_4);
}

 
/******************************************************************************
      函数说明：获取油门最小值此电调在占空比往上加时发生2短B音1低长B音
                并判断电机最小值
      入口数据：temp 电机位数
                
      返回值：  无
******************************************************************************/
void Unlock_pwm(uint8_t temp)
{
    int16_t pwm = 700;
    while (pwm <= (count_MAX*0.6))
    {
        switch (temp)
        {
            case 1:
            {
                // SEGGER_RTT_printf(0,"%d\r\n",pwm);
                printf("pwm1 = %d\r\n",pwm);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,pwm);
                pwm = pwm +10;
                HAL_Delay(500);
                break;
            }
            case 2:
            {
                // SEGGER_RTT_printf(0,"%d\r\n",pwm);
                printf("pwm2 = %d\r\n",pwm);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,pwm);
                pwm = pwm +10;
                HAL_Delay(500);
                break;
            }
            case 3:
            {
                // SEGGER_RTT_printf(0,"%d\r\n",pwm);
                printf("pwm3 = %d\r\n",pwm);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,pwm);
                pwm = pwm +10;
                HAL_Delay(500);
                break;
            }
            case 4:
            {
                // SEGGER_RTT_printf(0,"%d\r\n",pwm);
                printf("pwm4 = %d\r\n",pwm);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,pwm);
                pwm = pwm +10;
                HAL_Delay(500);
                break;
            }
        }
    }
}

/******************************************************************************
      函数说明：设置电机最大值
      入口数据：temp 电机位数
                
      返回值：  无
******************************************************************************/
void Set_MOTOR_MAX(uint16_t temp)
{	
    u8 key = 0; 
    int pwm1 ;
	int pwm2 ;
	int pwm3 ;
	int pwm4 ;
    
    switch (temp)
    {
        case 0:
        {
            //key=KEY_Scan(0);
            if (key == 1)
            {
                pwm1 = 1800;
                pwm2 = 1800;
                pwm3 = 1800;
                pwm4 = 1800;
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,pwm1);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,pwm2);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,pwm3);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,pwm4);
            }
            if (key == 2)
            {
                pwm1 = 1100;
                pwm2 = 1100;
                pwm3 = 1100;
                pwm4 = 1100; 
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,pwm1);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,pwm2);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,pwm3);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,pwm4);
            }
            HAL_Delay(500);
            break;
        }
        case 1:
        {
            while (1)
            {
                //key=KEY_Scan(0);
                // SEGGER_RTT_printf(0,"pwm1   ");
                // SEGGER_RTT_printf(0,"%d\r\n",pwm1);
                printf("pwm1 = %d\r\n",pwm1);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,pwm1);
                pwm1 = pwm1 + 10;
                HAL_Delay(500);
                if (key != 0)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,MOTOR1_PWM_MIN);
                    break;
                }
                if (pwm1 >= MOTOR1_PWM_MAX)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,MOTOR1_PWM_MIN);
                    break;
                }
            }
            break;
        }
        case 2:
        {
            while (1)
            {
                //key=KEY_Scan(0);
                // SEGGER_RTT_printf(0,"pwm2   ");
                // SEGGER_RTT_printf(0,"%d\r\n",pwm2);
                printf("pwm2 = %d\r\n",pwm2);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,pwm2);
                pwm2 = pwm2 + 10;
                HAL_Delay(500);
                if (key != 0)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,MOTOR2_PWM_MIN);
                    break;
                }
                if (pwm2 >= MOTOR2_PWM_MAX)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,MOTOR2_PWM_MIN);
                    break;
                }
            }
            break;
        }
        case 3:
        {
            while (1)
            {
                //key=KEY_Scan(0);
                // SEGGER_RTT_printf(0,"pwm3   ");
                // SEGGER_RTT_printf(0,"%d\r\n",pwm3);
                printf("pwm3 = %d\r\n",pwm3);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,pwm3);
                pwm3 = pwm3 + 10;
                HAL_Delay(500);
                if (key != 0)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,MOTOR3_PWM_MIN);
                    break;
                }
                if (pwm3 >= MOTOR3_PWM_MAX)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,MOTOR3_PWM_MIN);
                    break;
                }
            }
            break;
        }
        case 4:
        {
            while (1)
            {
                //key=KEY_Scan(0);
                // SEGGER_RTT_printf(0,"pwm4   ");
                // SEGGER_RTT_printf(0,"%d\r\n",pwm4);
                printf("pwm4 = %d\r\n",pwm4);
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,pwm4);
                pwm4 = pwm4 + 10;
                HAL_Delay(500);
                if (key != 0)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,MOTOR4_PWM_MIN);
                    break;
                }
                if (pwm4 >= MOTOR4_PWM_MAX)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,MOTOR4_PWM_MIN);
                    break;
                }
            }
            break;
        }
    }
}

/******************************************************************************
      函数说明：电机判断
				
      入口数据：val:最大值 temp：电机位号   mode:模式 1（判断解锁）、2(判断范围)
                
      返回值：  无
******************************************************************************/
void Motor_judgment(u16 val,int temp,int mode)
{
    int i;
    if (mode == 1)
    {
        for ( i = 100; i < val; i++)
        {
            switch (temp)
            {
                case 1:
                {   
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(200);
                    break;
                }
                case 2:
                {   
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(200);
                    break;
                }
                case 3:
                {     
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(200);
                    break;
                }
                case 4:
                {     
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(200);
                    break;
                }
                default:
                    break;
            }
        }
    }
    else if (mode == 2)
    {
        switch (temp)
        {
            case 1:
            {     
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,MOTOR1_MIN);
                HAL_Delay(1500);
                HAL_Delay(1500);
                for ( i = MOTOR1_MIN; i < val; i++)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(500);
                }
                break;
            }
            case 2:
            {     
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,MOTOR2_MIN);
                HAL_Delay(1500);
                HAL_Delay(1500);
                for ( i = MOTOR2_MIN; i < val; i++)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(500);
                }
                break;
            }
            case 3:
            {     
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,MOTOR3_MIN);
                HAL_Delay(1500);
                HAL_Delay(1500);
                for ( i = MOTOR3_MIN; i < val; i++)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(500);
                }
                break;
            }
            case 4:
            {     
                __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,MOTOR4_MIN);
                HAL_Delay(1500);
                HAL_Delay(1500);
                for ( i = MOTOR4_MIN; i < val; i++)
                {
                    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,i);
                    printf("i = %d\r\n",i);
                    HAL_Delay(500);
                }
                break;
            }
            default:
                break;
        }
    }
}

/******************************************************************************
      函数说明：解锁
      入口数据：无
                
      返回值：  无
******************************************************************************/
void Motor_Unlock(void)
{
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,MOTOR1_MIN);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,MOTOR2_MIN);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,MOTOR3_MIN);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,MOTOR4_MIN);
    HAL_Delay(1500);
    HAL_Delay(1500);
}


/******************************************************************************
      函数说明：运动方向
				
      入口数据：无
                
      返回值：  无
******************************************************************************/
void sport_motor(int16_t pwm1,int16_t pwm2,int16_t pwm3,int16_t pwm4)
{
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,pwm1);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,pwm2);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,pwm3);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,pwm4);
}

void MOTOR_TEXT()
{
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_1,MOTOR_PWM_TEXT);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_2,MOTOR_PWM_TEXT);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_3,MOTOR_PWM_TEXT);
    __HAL_TIM_SET_COMPARE(&ESC_PWM_LINE,TIM_CHANNEL_4,MOTOR_PWM_TEXT);
}

