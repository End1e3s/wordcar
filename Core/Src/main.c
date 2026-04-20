/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp, Ki, Kd;              
    float error, last_error, integral; 
    float output;                  
} PID_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PWM 10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RX_BUF_SIZE 64

// DMA 接收缓冲区
uint8_t rx1_buffer[RX_BUF_SIZE]; // 视觉模块数据
uint8_t rx3_buffer[RX_BUF_SIZE]; // JY901 数据

// 核心状态变量
float current_yaw = 0.0f; 
float target_yaw = 0.0f;  
float vision_target = 0.0f;   
volatile float base_speed = 0.0f;   // 基础直行速度
volatile long long total_pulses = 0; // 总里程
uint8_t run_state = 0;              // 状态机

// PID 结构体 
PID_TypeDef pid_left  = {150.0f, 10.0f, 0.0f, 0, 0, 0, 0}; 
PID_TypeDef pid_right = {150.0f, 10.0f, 0.0f, 0, 0, 0, 0};
PID_TypeDef pid_angle = {0.8f, 0.0f, 0.0f, 0, 0, 0, 0}; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Motor_SetSpeed(uint8_t motor_id, int pwm_val);
int Read_Speed(TIM_HandleTypeDef *htim);
float PID_Calc(PID_TypeDef *pid, float target, float current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    //开启电机 PWM 和编码器
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    //开启串口DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, RX_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx3_buffer, RX_BUF_SIZE);

    //死等 2 秒
    HAL_Delay(2000); 

    //锁定起跑线
    target_yaw = current_yaw; 

    //彻底清空 PID 历史残留
    pid_left.integral = 0; pid_left.last_error = 0;
    pid_right.integral = 0; pid_right.last_error = 0;
    pid_angle.integral = 0; pid_angle.last_error = 0;
    // 【新增】：开启云台舵机定时器 (假设你配置的是 TIM1 的 CH1)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    // 【新增】：一上电，立刻给 1580，让云台死死锁在正前方，不许乱动！
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1580); 
    run_state = 1; 

    // 6. 【最致命的一步】：一切就绪后，最后再把定时器大脑打开！！！
    HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 临时加两个全局变量来偷窥中断里的速度，记得写在 while 的上面
  extern volatile int speed_left;
  extern volatile int speed_right; 
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      switch (run_state) {
          case 1: // 【状态1：起步直行，看右侧立牌】
              base_speed = 10.0f; 
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1600); 
              if (total_pulses > 6000) {
                  pid_left.integral = 0; pid_right.integral = 0;                  
                  run_state = 2; 
              }
              break;
              
          case 2: // 【状态2：第一次停泊，准备左转90度】
              base_speed = 0.0f; 
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1580); 
              HAL_Delay(1000); // 刹车彻底停稳
              
              // 【核心】：严格在绝对数学坐标系上加90度
              target_yaw += 90.0f; 
              if(target_yaw > 180.0f) target_yaw -= 360.0f;
              if(target_yaw < -180.0f) target_yaw += 360.0f;
              
              pid_angle.error = 0; pid_angle.last_error = 0; pid_angle.integral = 0;
              pid_left.integral = 0; pid_right.integral = 0; 
              run_state = 3; 
              break;
              
          case 3: { // 【状态3：第一次原地旋转90度】
              base_speed = 0.0f; 
              
              float diff1 = target_yaw - current_yaw;
              if (diff1 > 180.0f) diff1 -= 360.0f;
              if (diff1 < -180.0f) diff1 += 360.0f;
              
              // 【修复】：容差收紧到 3 度
              if (fabs(diff1) < 3.0f) { 
                  HAL_Delay(800); // 等待车身彻底停止晃动
                  
                  // 【极其关键】：停稳后，把后台瞎算的旧误差全部清空，干干净净交棒！
                  // 绝对不许重置 target_yaw，死死锁住 90 度网格！
                  pid_angle.error = 0; pid_angle.last_error = 0; pid_angle.integral = 0;
                  pid_left.integral = 0; pid_right.integral = 0; 
                  
                  total_pulses = 0; 
                  run_state = 4;
              }
              break;
          }
              
          case 4: // 【状态4：横向逼近短直线】
              base_speed = 8.0f; // 慢点开，给足视觉模块反应时间
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1580); 
              
              // 这里的 2000 脉冲是横向距离，你需要下地试一下长短
              if (total_pulses > 2000) { 
                  pid_left.integral = 0; pid_right.integral = 0;
                  run_state = 5;
              }
              break;

          case 5: // 【状态5：第二次停泊，准备再左转90度】
              base_speed = 0.0f; 
              HAL_Delay(1000); 
              
              target_yaw += 90.0f; 
              if(target_yaw > 180.0f) target_yaw -= 360.0f;
              if(target_yaw < -180.0f) target_yaw += 360.0f;
              
              pid_angle.error = 0; pid_angle.last_error = 0; pid_angle.integral = 0;
              pid_left.integral = 0; pid_right.integral = 0; 
              run_state = 6; 
              break;

          case 6: { // 【状态6：第二次原地旋转90度】
              base_speed = 0.0f; 
              
              float diff2 = target_yaw - current_yaw;
              if (diff2 > 180.0f) diff2 -= 360.0f;
              if (diff2 < -180.0f) diff2 += 360.0f;
              
              if (fabs(diff2) < 3.0f) { 
                  HAL_Delay(800); 
                  
                  // 同样：清空误差，保持绝对正交网格
                  pid_angle.error = 0; pid_angle.last_error = 0; pid_angle.integral = 0;
                  pid_left.integral = 0; pid_right.integral = 0; 
                  
                  total_pulses = 0; 
                  run_state = 7;
              }
              break;
          }

          case 7: // 【状态7：U型掉头完成，直行返程】
              base_speed = 10.0f;
              // 队友这招妙啊！车掉了个头（朝南），物块在场地左边，刚好在车的右边！
              // 所以云台继续保持看右侧，完美复用！
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1600); 
              
              if (total_pulses > 6000) { 
                  pid_left.integral = 0; pid_right.integral = 0;
                  run_state = 8;
              }
              break;
              
          case 8: // 【状态8：彻底收工】
              base_speed = 0.0f;
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1580); 
              break;
      }

      HAL_Delay(10); 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ==========================================
// 1. 串口 DMA 空闲中断回调 (数据收完一帧自动触发)
// ==========================================
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // --- 解析 JY901 (USART3) ---
    if(huart->Instance == USART3) {
        if (Size >= 11) {
            for(int i = 0; i <= Size - 11; i++) {
                // 1. 找帧头
                if(rx3_buffer[i] == 0x55 && rx3_buffer[i+1] == 0x53) {
                    
                    // 【关键修复】：2. 算校验和，严防脏数据！
                    uint8_t sum = 0;
                    for(int j = 0; j < 10; j++) {
                        sum += rx3_buffer[i+j];
                    }
                    
                    // 3. 只有校验和通过，才更新角度
                    if (sum == rx3_buffer[i+10]) {
                        short raw_yaw = (short)((rx3_buffer[i+7] << 8) | rx3_buffer[i+6]);
                        current_yaw = (float)raw_yaw / 32768.0f * 180.0f;
                        break; 
                    }
                }
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx3_buffer, RX_BUF_SIZE);
    }
    // --- 解析视觉模块 (USART1) ---
    else if(huart->Instance == USART1) {
        // 假设协议是 A5 数据 5A
        if(rx1_buffer[0] == 0xA5 && rx1_buffer[Size-1] == 0x5A) {
            vision_target = (float)rx1_buffer[1]; 
        }
        // 重新开启 DMA
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, RX_BUF_SIZE);
    }
}
// ==========================================
// 补丁 2：串口错误回调函数 (DMA 护航神技)
// ==========================================
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 如果 JY901 串口因为溢出等原因死机了，立刻重启 DMA 接收！
    if(huart->Instance == USART3) {
        HAL_UART_AbortReceive(huart); // 先强行终止
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx3_buffer, RX_BUF_SIZE); // 重新开启
    }
    // 视觉串口同理
    if(huart->Instance == USART1) {
        HAL_UART_AbortReceive(huart); 
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, RX_BUF_SIZE); 
    }
}
// 2. 定时器 10ms 中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) 
    {
        // 1. 读速度 & 累加里程
        int speed_left  = Read_Speed(&htim2);
        int speed_right = -Read_Speed(&htim4); 
        total_pulses += (speed_left + speed_right) / 2;

        // 2. 角度环 PID (算出需要补偿的转向差速)
        float turn_adjust = 0.0f;
        float angle_err = target_yaw - current_yaw;
        
        // 处理过零点 (保证偏航纠正走捷径)
        if (angle_err > 180.0f)  angle_err -= 360.0f;
        if (angle_err < -180.0f) angle_err += 360.0f;
        
        // 动态限幅：平时直行可以给 30 的力度纠偏，但掉头时必须极其温柔！
        // 算出初始转向补偿
        // 动态分配 PID 参数和限幅，解决鱼和熊掌的问题！
        float max_turn; 
        
        if (run_state == 3 || run_state == 6) {
            // 【原地转向模式】降速防脱轨！转太快陀螺仪会跟不上！
            pid_angle.Kp = 1.0f;  
            pid_angle.Kd = 0.8f;  
            max_turn = 8.0f;      // 【核心】：从12降到8，宁可转慢点，也要它一次性精准卡在90度
        } else {
            // 【直行模式】暴力锁死直线，专治“稍微偏左”
            pid_angle.Kp = 1.0f;  // 加大纠偏力度，偏了硬拉回来
            pid_angle.Kd = 0.5f;  // 增加阻尼防S型
            max_turn = 15.0f;     // 给足油门权限
        }

        turn_adjust = PID_Calc(&pid_angle, 0.0f, -angle_err); 

        // 转向限幅
        if(turn_adjust > max_turn) turn_adjust = max_turn;
        if(turn_adjust < -max_turn) turn_adjust = -max_turn;
        // 3. 速度环 PID (基础速度 转向补偿)
        float target_L = base_speed - turn_adjust; 
        float target_R = base_speed + turn_adjust;

        float pwm_L = PID_Calc(&pid_left, target_L, speed_left);
        float pwm_R = PID_Calc(&pid_right, target_R, speed_right);

        // 4. 输出给电机
        Motor_SetSpeed(1, (int)pwm_L);
        Motor_SetSpeed(2, (int)pwm_R);
    }
}

void Motor_SetSpeed(uint8_t motor_id, int pwm_val)
{

	if (pwm_val > 5)       pwm_val += 2500;
	else if (pwm_val < -5) pwm_val -= 2500;
	else                    pwm_val = 0; // ̫С��������������
    // �޷���������ֹ PWM ������ʱ������
    if(pwm_val > MAX_PWM) pwm_val = MAX_PWM;
    if(pwm_val < -MAX_PWM) pwm_val = -MAX_PWM;

    if (motor_id == 1) // ��� A (����)
    {
        if (pwm_val >= 0) {
            // ��ת
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        } else {
            // ��ת (������ʱ�Զ�����)
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -pwm_val);
        }
    }
    else if (motor_id == 2) // ��� B (����)
    {
		pwm_val=-pwm_val;
        if (pwm_val >= 0) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_val);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -pwm_val);
        }
    }
}

// === ��������ȡ�������ٶ� ===
// ����ֵ�������ϴε��ú�ת����������
int Read_Speed(TIM_HandleTypeDef *htim)
{
    // ��ȡ����ֵ��ǿ��תΪ�з��Ŷ����� (���� 65535 -> -1 �ķ�ת���)
    int16_t speed = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    
    // �����������׼����һ�μ���
    __HAL_TIM_SET_COUNTER(htim, 0); 
    
    return (int)speed;
}

// === ������PID ���� ===
float PID_Calc(PID_TypeDef *pid, float target, float current)
{
    // 1. �������
    pid->error = target - current;
    
    // 2. ������ (�ۼ����)
    pid->integral += pid->error;
    // 2. �����޷� (һ��Ҫ�ӣ���Ȼ���ղ��Ի��ת)
	if(pid->integral > 10000) pid->integral = 10000;
	if(pid->integral < -10000) pid->integral = -10000;
    // 3. ΢���� (���仯��)
    float derivative = pid->error - pid->last_error;
    
    // 4. ���������
    pid->output = (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    
    // 5. ��¼���´�ʹ��
    pid->last_error = pid->error;
    
    return pid->output;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
