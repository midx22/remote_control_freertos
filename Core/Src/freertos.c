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
#include "adc.h"
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

// ADC Ping-Pong DMA事件标志
osEventFlagsId_t adc_pingpong_event_flags;
#define PING_BUFFER_READY    (1 << 0)  // Ping缓冲区数据就绪
#define PONG_BUFFER_READY    (1 << 1)  // Pong缓冲区数据就绪
#define DMA_ERROR_FLAG       (1 << 2)  // DMA错误标志

// ADC全局变量声明
volatile uint16_t ch0_avg, ch1_avg, ch2_avg, ch3_avg;

// ADC Ping-Pong双缓冲区 (每个缓冲区8个样本 = 4通道 * 2次采样)
#define PING_PONG_BUFFER_SIZE  8  // 4通道 * 2次采样
volatile uint16_t adc_ping_buffer[PING_PONG_BUFFER_SIZE];  // Ping缓冲区
volatile uint16_t adc_pong_buffer[PING_PONG_BUFFER_SIZE];  // Pong缓冲区
volatile uint16_t adc_pingpong_buffer[PING_PONG_BUFFER_SIZE * 2]; // 完整DMA缓冲区


// Ping-Pong状态管理
volatile uint8_t current_processing_buffer = 0; // 0=处理Ping, 1=处理Pong
volatile uint32_t ping_pong_count = 0;
volatile uint32_t data_ready_count = 0;
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
  .priority = (osPriority_t) osPriorityBelowNormal1,
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
  .priority = (osPriority_t) osPriorityAboveNormal7,
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

  // 创建ADC Ping-Pong事件标志
  adc_pingpong_event_flags = osEventFlagsNew(NULL);

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
  
  char buffer[25];
  uint32_t flags;

  
  // 初始化Ping-Pong缓冲区
  for(int i = 0; i < PING_PONG_BUFFER_SIZE * 2; i++) {
    adc_pingpong_buffer[i] = 0;
  }
  
  // 启动DMA连续转换，使用双倍大小的缓冲区实现Ping-Pong
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_pingpong_buffer, PING_PONG_BUFFER_SIZE * 2);
  
  
  /* Infinite loop */
  for(;;)
  {
    // 等待缓冲区数据就绪事件，超时时间1秒
    flags = osEventFlagsWait(adc_pingpong_event_flags, 
                            PING_BUFFER_READY | PONG_BUFFER_READY | DMA_ERROR_FLAG,
                            osFlagsWaitAny, 1000);
    
    if (flags & PING_BUFFER_READY) {
      // Ping缓冲区数据就绪，处理前半部分数据
      // 计算每个通道的平均值（2次采样）
      ch0_avg = (adc_pingpong_buffer[0] + adc_pingpong_buffer[4]) / 2;
      ch1_avg = (adc_pingpong_buffer[1] + adc_pingpong_buffer[5]) / 2;
      ch2_avg = (adc_pingpong_buffer[2] + adc_pingpong_buffer[6]) / 2;
      ch3_avg = (adc_pingpong_buffer[3] + adc_pingpong_buffer[7]) / 2;
      
      current_processing_buffer = 0; // 正在处理Ping
      
    } else if (flags & PONG_BUFFER_READY) {
      // Pong缓冲区数据就绪，处理后半部分数据
      // 计算每个通道的平均值（2次采样）
      ch0_avg = (adc_pingpong_buffer[8] + adc_pingpong_buffer[12]) / 2;
      ch1_avg = (adc_pingpong_buffer[9] + adc_pingpong_buffer[13]) / 2;
      ch2_avg = (adc_pingpong_buffer[10] + adc_pingpong_buffer[14]) / 2;
      ch3_avg = (adc_pingpong_buffer[11] + adc_pingpong_buffer[15]) / 2;
      
      current_processing_buffer = 1; // 正在处理Pong
      
    } 
      
    
    
    // 显示处理后的数据
    sprintf(buffer, "CH0:%04u", ch0_avg);
    OLED_ShowString(1, 1, buffer);
    sprintf(buffer, "CH1:%04u", ch1_avg);
    OLED_ShowString(2, 1, buffer);
    sprintf(buffer, "CH2:%04u", ch2_avg);
    OLED_ShowString(3, 1, buffer);
    sprintf(buffer, "CH3:%04u", ch3_avg);
    OLED_ShowString(4, 1, buffer);

  
    data_ready_count++;
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
  char packet_a[20];
  char packet_b[20];
  
  // nRF24L01+ 地址配置
  uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  
  // 初始化nRF24L01+
  NRF24_GPIO_Init();
  osDelay(100);
  
  if (NRF24_TestConnection()) {
    NRF24_Init();
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
    NRF24_FlushTx();
    NRF24_FlushRx();
    osDelay(10);
    NRF24_SetTxModeDynamic(address);
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
  }
  
  /* Infinite loop */
  for(;;)
  {
    // 发送A包：CH0和CH1数据
    sprintf(packet_a, "A:%04d,%04d", ch0_avg, ch1_avg);
    NRF24_TransmitDynamic((uint8_t*)packet_a, strlen(packet_a));
    osDelay(400);
    
    // 发送B包：CH2和CH3数据
    sprintf(packet_b, "B:%04d,%04d", ch2_avg, ch3_avg);
    NRF24_TransmitDynamic((uint8_t*)packet_b, strlen(packet_b));
    osDelay(400);  // 总周期1.5秒
  }
  /* USER CODE END Startwireless_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  ADC DMA转换完成回调函数 (Ping-Pong Pong缓冲区就绪)
 * @param  hadc: ADC句柄
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    // DMA完全转换完成 = Pong缓冲区（后半部分）数据就绪
    ping_pong_count++;
    
    // 设置Pong缓冲区就绪事件标志
    if (adc_pingpong_event_flags != NULL) {
      osEventFlagsSet(adc_pingpong_event_flags, PONG_BUFFER_READY);
    }
  }
}

/**
 * @brief  ADC DMA半传输完成回调函数 (Ping-Pong Ping缓冲区就绪)
 * @param  hadc: ADC句柄
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    // DMA半传输完成 = Ping缓冲区（前半部分）数据就绪
    ping_pong_count++;
    
    // 设置Ping缓冲区就绪事件标志
    if (adc_pingpong_event_flags != NULL) {
      osEventFlagsSet(adc_pingpong_event_flags, PING_BUFFER_READY);
    }
  }
}

/**
 * @brief  ADC错误回调函数
 * @param  hadc: ADC句柄
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    // 设置DMA错误事件标志
    if (adc_pingpong_event_flags != NULL) {
      osEventFlagsSet(adc_pingpong_event_flags, DMA_ERROR_FLAG);
    }
  }
}

/* USER CODE END Application */

