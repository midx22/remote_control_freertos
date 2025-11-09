/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <stdio.h>
#include <string.h>
#include "bsp_spi.h"  
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// nRF24L01+事件标志用于中断通信
osEventFlagsId_t nrf24_event_flags;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLED_Task */
osThreadId_t OLED_TaskHandle;
const osThreadAttr_t OLED_Task_attributes = {
  .name = "OLED_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC_Task */
osThreadId_t ADC_TaskHandle;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for wireless_Task */
osThreadId_t wireless_TaskHandle;
const osThreadAttr_t wireless_Task_attributes = {
  .name = "wireless_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartOLED_Task(void *argument);
void StartADC_Task(void *argument);
void Startwireless_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // 创建nRF24L01+事件标志
  nrf24_event_flags = osEventFlagsNew(NULL);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OLED_Task */
  OLED_TaskHandle = osThreadNew(StartOLED_Task, NULL, &OLED_Task_attributes);

  /* creation of ADC_Task */
  ADC_TaskHandle = osThreadNew(StartADC_Task, NULL, &ADC_Task_attributes);

  /* creation of wireless_Task */
  wireless_TaskHandle = osThreadNew(Startwireless_Task, NULL, &wireless_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartOLED_Task */
/**
* @brief Function implementing the OLED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLED_Task */
void StartOLED_Task(void *argument)
{
  /* USER CODE BEGIN StartOLED_Task */
  uint32_t counter = 0;
  char buffer[20];
  
  /* Infinite loop */
  for(;;)
  {
    // 更新计数器显示

    osDelay(1000); // 每秒更新一次
  }
  /* USER CODE END StartOLED_Task */
}

/* USER CODE BEGIN Header_StartADC_Task */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC_Task */
void StartADC_Task(void *argument)
{
  /* USER CODE BEGIN StartADC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartADC_Task */
}

/* USER CODE BEGIN Header_Startwireless_Task */
/**
* @brief Function implementing the wireless_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startwireless_Task */
void Startwireless_Task(void *argument)
{
  /* USER CODE BEGIN Startwireless_Task */
  uint8_t connection_status = 0;
  uint8_t send_result = 0;
  char display_buffer[20];
  uint32_t send_counter = 0;
  uint8_t status_reg = 0;
  uint8_t message_index = 0;
  
  // 不同长度的测试消息
  char test_messages[][25] = {
    "Hi!",                    // 3 字节
    "Hello",                  // 5 字节  
    "Test123",                // 7 字节

  };
  uint8_t num_messages = sizeof(test_messages) / sizeof(test_messages[0]);
  
  // nRF24L01+ 地址配置（5字节地址）
  uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  
  // 初始化nRF24L01+ GPIO
  NRF24_GPIO_Init();
  
  // 短暂延时让硬件稳定
  osDelay(100);
  
  // 测试连接
  connection_status = NRF24_TestConnection();
  if (connection_status) {
    OLED_ShowString(1, 1, "nRF24: OK    ");
    
    // 初始化nRF24L01+
    NRF24_Init();
    
    // 完全清除所有状态标志
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
    NRF24_FlushTx();
    NRF24_FlushRx();
    
    osDelay(10);
    
    // 设置为动态长度发送模式
    NRF24_SetTxModeDynamic(address);
    
    // 再次清除状态
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);


    
  }
  
  /* Infinite loop */
  for(;;)
  {
    if (connection_status) {
      
      // 获取当前要发送的消息
      char* current_msg = test_messages[message_index];
      uint8_t msg_length = strlen(current_msg);
      
      // 发送数据
      send_result = NRF24_TransmitDynamic((uint8_t*)current_msg, msg_length);
      
      if (send_result) {

        // 显示发送的消息（前16字符）
        char temp_str[17] = {0};
        strncpy(temp_str, current_msg, 16);
        temp_str[16] = '\0';
        OLED_Clear();
        OLED_ShowString(3, 1, temp_str);

      } else {
        // 显示发送失败
        sprintf(display_buffer, "TX FAIL %dB", msg_length);
        OLED_ShowString(2, 2, display_buffer);
        OLED_ShowString(3, 2, "Check receiver");
        OLED_ShowString(4, 2, "             ");
      }
      
      // 切换到下一条消息
      message_index = (message_index + 1) % num_messages;
      
      // 每2秒发送一次
      osDelay(2000);
      
    } else {
      // 连接失败，延时后重试
      osDelay(1000);
    }
  }
  /* USER CODE END Startwireless_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

