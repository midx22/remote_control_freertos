#ifndef __bsp_spi_H
#define __bsp_spi_H

#include "main.h"
#include "spi.h"

// nRF24L01+ 寄存器定义
#define NRF24_REG_CONFIG        0x00
#define NRF24_REG_EN_AA         0x01
#define NRF24_REG_EN_RXADDR     0x02
#define NRF24_REG_SETUP_AW      0x03
#define NRF24_REG_SETUP_RETR    0x04
#define NRF24_REG_RF_CH         0x05
#define NRF24_REG_RF_SETUP      0x06
#define NRF24_REG_STATUS        0x07
#define NRF24_REG_OBSERVE_TX    0x08
#define NRF24_REG_RPD           0x09
#define NRF24_REG_RX_ADDR_P0    0x0A
#define NRF24_REG_RX_ADDR_P1    0x0B
#define NRF24_REG_RX_ADDR_P2    0x0C
#define NRF24_REG_RX_ADDR_P3    0x0D
#define NRF24_REG_RX_ADDR_P4    0x0E
#define NRF24_REG_RX_ADDR_P5    0x0F
#define NRF24_REG_TX_ADDR       0x10
#define NRF24_REG_RX_PW_P0      0x11
#define NRF24_REG_RX_PW_P1      0x12
#define NRF24_REG_RX_PW_P2      0x13
#define NRF24_REG_RX_PW_P3      0x14
#define NRF24_REG_RX_PW_P4      0x15
#define NRF24_REG_RX_PW_P5      0x16
#define NRF24_REG_FIFO_STATUS   0x17

// 动态载荷相关寄存器
#define NRF24_REG_DYNPD         0x1C
#define NRF24_REG_FEATURE       0x1D

// SPI命令定义
#define NRF24_CMD_R_REGISTER    0x00  // 读寄存器
#define NRF24_CMD_W_REGISTER    0x20  // 写寄存器
#define NRF24_CMD_R_RX_PAYLOAD  0x61  // 读接收载荷
#define NRF24_CMD_W_TX_PAYLOAD  0xA0  // 写发送载荷
#define NRF24_CMD_FLUSH_TX      0xE1  // 清空发送FIFO
#define NRF24_CMD_FLUSH_RX      0xE2  // 清空接收FIFO
#define NRF24_CMD_REUSE_TX_PL   0xE3  // 重复使用发送载荷
#define NRF24_CMD_R_RX_PL_WID   0x60  // 读取接收载荷长度
#define NRF24_CMD_ACTIVATE      0x50  // 激活特性
#define NRF24_CMD_NOP           0xFF  // 空操作

// 寄存器位定义
#define NRF24_CONFIG_MASK_RX_DR  0x40  // RX_DR中断屏蔽
#define NRF24_CONFIG_MASK_TX_DS  0x20  // TX_DS中断屏蔽
#define NRF24_CONFIG_MASK_MAX_RT 0x10  // MAX_RT中断屏蔽
#define NRF24_CONFIG_EN_CRC      0x08  // 使能CRC
#define NRF24_CONFIG_CRCO        0x04  // CRC长度
#define NRF24_CONFIG_PWR_UP      0x02  // 上电
#define NRF24_CONFIG_PRIM_RX     0x01  // 接收模式

#define NRF24_STATUS_RX_DR       0x40  // 接收数据就绪
#define NRF24_STATUS_TX_DS       0x20  // 发送数据完成
#define NRF24_STATUS_MAX_RT      0x10  // 最大重传次数
#define NRF24_STATUS_RX_P_NO     0x0E  // 接收管道号
#define NRF24_STATUS_TX_FULL     0x01  // 发送FIFO满

#define NRF24_FIFO_STATUS_TX_REUSE 0x40  // 重复使用发送载荷
#define NRF24_FIFO_STATUS_TX_FULL  0x20  // 发送FIFO满
#define NRF24_FIFO_STATUS_TX_EMPTY 0x10  // 发送FIFO空
#define NRF24_FIFO_STATUS_RX_FULL  0x02  // 接收FIFO满
#define NRF24_FIFO_STATUS_RX_EMPTY 0x01  // 接收FIFO空

// GPIO引脚定义 (根据实际连接配置)
#define NRF24_CS_PIN            CEN_Pin          // PC0
#define NRF24_CS_PORT           CEN_GPIO_Port    // GPIOC
#define NRF24_CE_PIN            CE_Pin           // PE2  
#define NRF24_CE_PORT           CE_GPIO_Port     // GPIOE

// IRQ引脚定义
#define NRF24_IRQ_PIN           GPIO_PIN_1       // PC1
#define NRF24_IRQ_PORT          GPIOC            // GPIOC

// 控制宏
#define NRF24_CS_LOW()          HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_RESET)
#define NRF24_CS_HIGH()         HAL_GPIO_WritePin(NRF24_CS_PORT, NRF24_CS_PIN, GPIO_PIN_SET)
#define NRF24_CE_LOW()          HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET)
#define NRF24_CE_HIGH()         HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET)

// 函数声明
void NRF24_GPIO_Init(void);
uint8_t NRF24_SPI_Transfer(uint8_t data);
uint8_t NRF24_ReadRegister(uint8_t reg);
uint8_t NRF24_WriteRegister(uint8_t reg, uint8_t value);
void NRF24_ReadRegisterMulti(uint8_t reg, uint8_t* data, uint8_t length);
void NRF24_WriteRegisterMulti(uint8_t reg, const uint8_t* data, uint8_t length);

// 初始化和配置函数
uint8_t NRF24_TestConnection(void);
void NRF24_Init(void);
void NRF24_Reset(void);
void NRF24_SetTxMode(const uint8_t* address);
void NRF24_SetRxMode(const uint8_t* address, uint8_t payload_size);

// 发送和接收函数
uint8_t NRF24_Transmit(const uint8_t* data, uint8_t length);
uint8_t NRF24_Receive(uint8_t* data, uint8_t* length);
void NRF24_FlushTx(void);
void NRF24_FlushRx(void);
uint8_t NRF24_GetStatus(void);
void NRF24_ClearInterrupts(void);

// 状态查询函数
uint8_t NRF24_IsDataAvailable(void);
uint8_t NRF24_IsTxFifoFull(void);
uint8_t NRF24_IsRxFifoEmpty(void);

// 动态载荷函数
void NRF24_EnableDynamicPayload(void);
void NRF24_SetRxModeDynamic(const uint8_t* address);
uint8_t NRF24_ReceiveDynamic(uint8_t* data, uint8_t* length);
void NRF24_SetTxModeDynamic(const uint8_t* address);
uint8_t NRF24_TransmitDynamic(const uint8_t* data, uint8_t length);

// IRQ中断相关函数
void NRF24_EnableIRQ(void);
void NRF24_DisableIRQ(void);
uint8_t NRF24_IRQ_Handler(void);

#endif /* __BSP_SPI_H */