#include "bsp_spi.h"
#include "cmsis_os.h"
#include "dwt_delay.h"

extern SPI_HandleTypeDef hspi2; // 使用SPI2

/**
 * @brief 初始化nRF24L01+的GPIO引脚状态
 * @note GPIO引脚已在CubeMX中配置，这里只设置初始状态
 */
void NRF24_GPIO_Init(void)
{
    // 初始化引脚状态（GPIO已在CubeMX中配置）
    NRF24_CS_HIGH();  // CS默认高电平（未选中）
    NRF24_CE_LOW();   // CE默认低电平（待机模式）
}

/**
 * @brief SPI数据传输
 * @param data 要发送的数据
 * @retval 接收到的数据
 */
uint8_t NRF24_SPI_Transfer(uint8_t data)
{
    uint8_t rx_data = 0;
    
    // 使用HAL库进行SPI传输
    if (HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data, 1, 1000) != HAL_OK) {
        // SPI传输失败
        return 0xFF;
    }
    
    return rx_data;
}

/**
 * @brief 读取nRF24L01+寄存器
 * @param reg 寄存器地址
 * @retval 寄存器值
 */
uint8_t NRF24_ReadRegister(uint8_t reg)
{
    uint8_t result = 0;
    
    NRF24_CS_LOW();                                    // 拉低CS
    NRF24_SPI_Transfer(NRF24_CMD_R_REGISTER | reg);   // 发送读命令+寄存器地址
    result = NRF24_SPI_Transfer(0xFF);                 // 读取数据
    NRF24_CS_HIGH();                                   // 拉高CS
    
    return result;
}

/**
 * @brief 写入nRF24L01+寄存器
 * @param reg 寄存器地址
 * @param value 要写入的值
 * @retval 状态寄存器值
 */
uint8_t NRF24_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t status = 0;
    
    NRF24_CS_LOW();                                    // 拉低CS
    status = NRF24_SPI_Transfer(NRF24_CMD_W_REGISTER | reg); // 发送写命令+寄存器地址
    NRF24_SPI_Transfer(value);                         // 发送数据
    NRF24_CS_HIGH();                                   // 拉高CS
    
    return status;
}

/**
 * @brief 测试nRF24L01+连接
 * @retval 0-连接失败, 1-连接成功
 */
uint8_t NRF24_TestConnection(void)
{
    uint8_t test_value = 0x55;  // 测试值
    uint8_t read_value = 0;
    
    // 写入测试值到RF_CH寄存器（该寄存器可读写，默认值为2）
    NRF24_WriteRegister(NRF24_REG_RF_CH, test_value);
    
    // 短暂延时
    osDelay(1);
    
    // 读取RF_CH寄存器
    read_value = NRF24_ReadRegister(NRF24_REG_RF_CH);
    
    // 恢复默认值
    NRF24_WriteRegister(NRF24_REG_RF_CH, 0x02);
    
    // 检查读写是否一致
    if (read_value == test_value) {
        return 1;  // 通信成功
    } else {
        return 0;  // 通信失败
    }
}

/**
 * @brief 读取多字节寄存器
 * @param reg 寄存器地址
 * @param data 数据缓冲区
 * @param length 数据长度
 */
void NRF24_ReadRegisterMulti(uint8_t reg, uint8_t* data, uint8_t length)
{
    NRF24_CS_LOW();
    NRF24_SPI_Transfer(NRF24_CMD_R_REGISTER | reg);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = NRF24_SPI_Transfer(0xFF);
    }
    NRF24_CS_HIGH();
}

/**
 * @brief 写入多字节寄存器
 * @param reg 寄存器地址
 * @param data 数据缓冲区
 * @param length 数据长度
 */
void NRF24_WriteRegisterMulti(uint8_t reg, const uint8_t* data, uint8_t length)
{
    NRF24_CS_LOW();
    NRF24_SPI_Transfer(NRF24_CMD_W_REGISTER | reg);
    for (uint8_t i = 0; i < length; i++) {
        NRF24_SPI_Transfer(data[i]);
    }
    NRF24_CS_HIGH();
}

/**
 * @brief 初始化nRF24L01+
 */
void NRF24_Init(void)
{
    // 初始化GPIO
    NRF24_GPIO_Init();
    
    // 等待芯片启动
    osDelay(100);
    
    // 复位配置
    NRF24_Reset();
}

/**
 * @brief 复位nRF24L01+到默认配置
 */
void NRF24_Reset(void)
{
    NRF24_CE_LOW();
    
    // 清除状态标志
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
    
    // 清空FIFO
    NRF24_FlushTx();
    NRF24_FlushRx();
    
    // 基本配置：使能CRC，上电，发送模式
    NRF24_WriteRegister(NRF24_REG_CONFIG, 
                       NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP);
    
    // 关闭自动应答
    NRF24_WriteRegister(NRF24_REG_EN_AA, 0x00);
    
    // 使能接收管道0和1
    NRF24_WriteRegister(NRF24_REG_EN_RXADDR, 0x03);
    
    // 5字节地址宽度
    NRF24_WriteRegister(NRF24_REG_SETUP_AW, 0x03);
    
    // 关闭重传
    NRF24_WriteRegister(NRF24_REG_SETUP_RETR, 0x00);
    
    // 设置射频频道为2.402GHz
    NRF24_WriteRegister(NRF24_REG_RF_CH, 0x02);
    
    // 射频设置：1Mbps，0dBm
    NRF24_WriteRegister(NRF24_REG_RF_SETUP, 0x06);
    
    // 等待配置完成
    osDelay(5);
}

/**
 * @brief 设置为发送模式
 * @param address 发送地址（5字节）
 */
void NRF24_SetTxMode(const uint8_t* address)
{
    NRF24_CE_LOW();
    
    // 设置发送地址
    NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, address, 5);
    
    // 设置接收地址P0（用于接收自动应答）
    NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, address, 5);
    
    // 设置配置寄存器为发送模式
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    config &= ~NRF24_CONFIG_PRIM_RX;  // 清除PRIM_RX位（发送模式）
    NRF24_WriteRegister(NRF24_REG_CONFIG, config);
    
    DWT_Delay_us(150); // 模式切换需要至少130us
}

/**
 * @brief 设置为接收模式
 * @param address 接收地址（5字节）
 * @param payload_size 载荷大小
 */
void NRF24_SetRxMode(const uint8_t* address, uint8_t payload_size)
{
    NRF24_CE_LOW();
    
    // 设置接收地址P0（与发送保持一致）
    NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, address, 5);
    
    // 设置载荷长度
    NRF24_WriteRegister(NRF24_REG_RX_PW_P0, payload_size);
    
    // 设置配置寄存器为接收模式
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    config |= NRF24_CONFIG_PRIM_RX;  // 设置PRIM_RX位（接收模式）
    NRF24_WriteRegister(NRF24_REG_CONFIG, config);
    
    NRF24_CE_HIGH();  // 启动接收
    DWT_Delay_us(150); // 模式切换需要至少130us
}

/**
 * @brief 发送数据
 * @param data 数据缓冲区
 * @param length 数据长度
 * @retval 0-发送失败, 1-发送成功
 */
uint8_t NRF24_Transmit(const uint8_t* data, uint8_t length)
{
    uint32_t timeout = 1000;  // 超时计数
    
    // 清除状态标志
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
    
    // 写入发送载荷
    NRF24_CS_LOW();
    NRF24_SPI_Transfer(NRF24_CMD_W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        NRF24_SPI_Transfer(data[i]);
    }
    NRF24_CS_HIGH();
    
    // 脉冲CE启动发送
    NRF24_CE_HIGH();
    DWT_Delay_us(15);  // 至少10us
    NRF24_CE_LOW();
    
    // 等待发送完成
    while (timeout--) {
        uint8_t status = NRF24_GetStatus();
        
        if (status & NRF24_STATUS_TX_DS) {
            // 发送成功
            NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
            return 1;
        }
        
        if (status & NRF24_STATUS_MAX_RT) {
            // 发送失败（最大重传次数）
            NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
            NRF24_FlushTx();
            return 0;
        }
        
        osDelay(1);
    }
    
    return 0;  // 超时
}

/**
 * @brief 接收数据
 * @param data 数据缓冲区
 * @param length 接收到的数据长度
 * @retval 0-无数据, 1-有数据
 */
uint8_t NRF24_Receive(uint8_t* data, uint8_t* length)
{
    uint8_t status = NRF24_GetStatus();
    
    if (status & NRF24_STATUS_RX_DR) {
        // 有数据可读
        uint8_t pipe = (status & NRF24_STATUS_RX_P_NO) >> 1;
        
        if (pipe < 6) {  // 有效管道
            // 读取载荷长度
            *length = NRF24_ReadRegister(NRF24_REG_RX_PW_P0 + pipe);
            
            if (*length <= 32 && *length > 0) {  // 载荷长度有效
                // 读取载荷数据
                NRF24_CS_LOW();
                NRF24_SPI_Transfer(NRF24_CMD_R_RX_PAYLOAD);
                for (uint8_t i = 0; i < *length; i++) {
                    data[i] = NRF24_SPI_Transfer(0xFF);
                }
                NRF24_CS_HIGH();
                
                // 清除中断标志
                NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
                
                return 1;
            } else {
                // 长度无效，清除FIFO
                NRF24_FlushRx();
                NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
                return 0;
            }
        } else {
            // 无效管道，清除标志
            NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
            return 0;
        }
    }
    
    return 0;
}

/**
 * @brief 清空发送FIFO
 */
void NRF24_FlushTx(void)
{
    NRF24_CS_LOW();
    NRF24_SPI_Transfer(NRF24_CMD_FLUSH_TX);
    NRF24_CS_HIGH();
}

/**
 * @brief 清空接收FIFO
 */
void NRF24_FlushRx(void)
{
    NRF24_CS_LOW();
    NRF24_SPI_Transfer(NRF24_CMD_FLUSH_RX);
    NRF24_CS_HIGH();
}

/**
 * @brief 获取状态寄存器
 * @retval 状态寄存器值
 */
uint8_t NRF24_GetStatus(void)
{
    return NRF24_ReadRegister(NRF24_REG_STATUS);
}

/**
 * @brief 清除所有中断标志
 */
void NRF24_ClearInterrupts(void)
{
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
}

/**
 * @brief 检查是否有数据可读
 * @retval 0-无数据, 1-有数据
 */
uint8_t NRF24_IsDataAvailable(void)
{
    uint8_t status = NRF24_GetStatus();
    return (status & NRF24_STATUS_RX_DR) ? 1 : 0;
}

/**
 * @brief 检查发送FIFO是否满
 * @retval 0-未满, 1-已满
 */
uint8_t NRF24_IsTxFifoFull(void)
{
    uint8_t fifo_status = NRF24_ReadRegister(NRF24_REG_FIFO_STATUS);
    return (fifo_status & NRF24_FIFO_STATUS_TX_FULL) ? 1 : 0;
}

/**
 * @brief 检查接收FIFO是否空
 * @retval 0-非空, 1-空
 */
uint8_t NRF24_IsRxFifoEmpty(void)
{
    uint8_t fifo_status = NRF24_ReadRegister(NRF24_REG_FIFO_STATUS);
    return (fifo_status & NRF24_FIFO_STATUS_RX_EMPTY) ? 1 : 0;
}

/**
 * @brief 启用动态载荷长度功能
 */
void NRF24_EnableDynamicPayload(void)
{
    // 激活功能寄存器（某些nRF24L01+需要这个步骤）
    NRF24_CS_LOW();
    NRF24_SPI_Transfer(NRF24_CMD_ACTIVATE);
    NRF24_SPI_Transfer(0x73);
    NRF24_CS_HIGH();
    
    // 启用动态载荷长度功能
    NRF24_WriteRegister(NRF24_REG_FEATURE, 0x04);
    
    // 为所有管道启用动态载荷
    NRF24_WriteRegister(NRF24_REG_DYNPD, 0x3F);
    
    // 启用自动应答（动态载荷需要自动应答）
    NRF24_WriteRegister(NRF24_REG_EN_AA, 0x3F);
}

/**
 * @brief 设置动态长度接收模式
 * @param address 接收地址（5字节）
 */
void NRF24_SetRxModeDynamic(const uint8_t* address)
{
    NRF24_CE_LOW();
    
    // 设置接收地址到P1管道
    NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P1, address, 5);
    
    // 启用动态载荷
    NRF24_EnableDynamicPayload();
    
    // 设置为接收模式
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    config |= NRF24_CONFIG_PRIM_RX;  // 设置PRIM_RX位（接收模式）
    NRF24_WriteRegister(NRF24_REG_CONFIG, config);
    
    NRF24_CE_HIGH();  // 启动接收
    DWT_Delay_us(150);
}

/**
 * @brief 动态长度接收函数
 * @param data 数据缓冲区
 * @param length 返回实际接收到的数据长度
 * @retval 0-无数据, 1-有数据
 */
uint8_t NRF24_ReceiveDynamic(uint8_t* data, uint8_t* length)
{
    uint8_t status = NRF24_GetStatus();
    
    if (status & NRF24_STATUS_RX_DR) {
        // 读取动态载荷长度
        NRF24_CS_LOW();
        NRF24_SPI_Transfer(NRF24_CMD_R_RX_PL_WID);
        *length = NRF24_SPI_Transfer(0xFF);
        NRF24_CS_HIGH();
        
        // 检查长度有效性
        if (*length <= 32 && *length > 0) {
            // 读取实际数据
            NRF24_CS_LOW();
            NRF24_SPI_Transfer(NRF24_CMD_R_RX_PAYLOAD);
            for (uint8_t i = 0; i < *length; i++) {
                data[i] = NRF24_SPI_Transfer(0xFF);
            }
            NRF24_CS_HIGH();
            
            // 清除接收标志
            NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
            
            return 1; // 成功接收
        } else {
            // 长度无效，清除FIFO
            NRF24_CS_LOW();
            NRF24_SPI_Transfer(NRF24_CMD_FLUSH_RX);
            NRF24_CS_HIGH();
            
            NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
            return 0;
        }
    }
    
    return 0; // 无数据
}

/**
 * @brief 设置动态长度发送模式
 * @param address 发送地址（5字节）
 */
void NRF24_SetTxModeDynamic(const uint8_t* address)
{
    NRF24_CE_LOW();
    
    // 设置发送地址
    NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, address, 5);
    
    // 设置接收地址P0（用于接收自动应答）
    NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, address, 5);
    
    // 启用动态载荷
    NRF24_EnableDynamicPayload();
    
    // 设置配置寄存器为发送模式
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    config &= ~NRF24_CONFIG_PRIM_RX;  // 清除PRIM_RX位（发送模式）
    NRF24_WriteRegister(NRF24_REG_CONFIG, config);
    
    DWT_Delay_us(150); // 模式切换需要至少130us
}

/**
 * @brief 发送动态长度数据
 * @param data 数据缓冲区
 * @param length 实际数据长度（1-32字节）
 * @retval 0-失败, 1-成功
 */
uint8_t NRF24_TransmitDynamic(const uint8_t* data, uint8_t length)
{
    if (length > 32 || length == 0) {
        return 0; // 无效长度
    }
    
    uint32_t timeout = 1000;
    
    // 清除状态标志
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);
    
    // 写入载荷（长度由芯片自动检测）
    NRF24_CS_LOW();
    NRF24_SPI_Transfer(NRF24_CMD_W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        NRF24_SPI_Transfer(data[i]);
    }
    NRF24_CS_HIGH();
    
    // 启动发送
    NRF24_CE_HIGH();
    DWT_Delay_us(15);
    NRF24_CE_LOW();
    
    // 等待发送完成
    while (timeout--) {
        uint8_t status = NRF24_GetStatus();
        
        if (status & NRF24_STATUS_TX_DS) {
            NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
            return 1; // 成功
        }
        
        if (status & NRF24_STATUS_MAX_RT) {
            NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
            NRF24_FlushTx();
            return 0; // 失败
        }
        
        DWT_Delay_us(100);
    }
    
    return 0; // 超时
}

/**
 * @brief 配置nRF24L01+的IRQ中断
 */
void NRF24_EnableIRQ(void)
{
    // 清除所有中断屏蔽，使能所有中断
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    config &= ~(NRF24_CONFIG_MASK_RX_DR | NRF24_CONFIG_MASK_TX_DS | NRF24_CONFIG_MASK_MAX_RT);
    NRF24_WriteRegister(NRF24_REG_CONFIG, config);
}

/**
 * @brief 禁用nRF24L01+的IRQ中断
 */
void NRF24_DisableIRQ(void)
{
    // 屏蔽所有中断
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    config |= (NRF24_CONFIG_MASK_RX_DR | NRF24_CONFIG_MASK_TX_DS | NRF24_CONFIG_MASK_MAX_RT);
    NRF24_WriteRegister(NRF24_REG_CONFIG, config);
}

/**
 * @brief IRQ中断处理函数
 * @retval 返回中断类型：1-RX, 2-TX, 4-MAX_RT
 */
uint8_t NRF24_IRQ_Handler(void)
{
    uint8_t status = NRF24_GetStatus();
    uint8_t irq_type = 0;
    
    // 检查接收中断
    if (status & NRF24_STATUS_RX_DR) {
        irq_type |= 1;  // RX interrupt
        // 注意：不在这里清除RX_DR标志，在接收函数中清除
    }
    
    // 检查发送完成中断
    if (status & NRF24_STATUS_TX_DS) {
        irq_type |= 2;  // TX interrupt
        NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
    }
    
    // 检查最大重传中断
    if (status & NRF24_STATUS_MAX_RT) {
        irq_type |= 4;  // MAX_RT interrupt
        NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
        NRF24_FlushTx();
    }
    
    return irq_type;
}