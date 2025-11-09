#ifndef __DWT_DELAY_H
#define __DWT_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"

/**
 * @brief 初始化DWT计数器用于微秒延时
 * @note 需要在main函数中调用HAL_Init()后调用此函数
 */
void DWT_Delay_Init(void);

/**
 * @brief 微秒级精确延时
 * @param us 延时时间（微秒）
 * @note 这是一个忙等待延时，适用于短时间的精确延时
 *       在FreeRTOS中，对于短时间（通常<1ms）的延时可以使用
 */
void DWT_Delay_us(uint32_t us);

/**
 * @brief 毫秒级精确延时
 * @param ms 延时时间（毫秒）
 * @note 这是一个忙等待延时，适用于短时间的精确延时
 */
void DWT_Delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __DWT_DELAY_H */