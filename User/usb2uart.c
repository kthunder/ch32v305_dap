#include "usb2uart.h"

// 添加回调函数指针
static uint32_t dma_tx_length = 0;
// DMA发送缓冲区和状态
static uint8_t uart3_tx_buffer[256];
static volatile uint8_t uart3_dma_tx_busy = 0;
static uint8_t uart3_rx_buffer[UART_DMA_BUF_LEN];
static volatile uint16_t uart3_rx_write_pos = 0;
extern chry_ringbuffer_t g_uartrx; // 假设这个环形缓冲区已定义
void uart_init()
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    USART_InitTypeDef USART_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // 配置GPIO
    // PB10: USART3_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // PB11: USART3_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置USART3
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);

    // 启用USART3空闲中断用于接收处理
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void uart3_dma_init(void)
{
    DMA_InitTypeDef DMA_InitStructure = { 0 };
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /*********** USART3_TX DMA CONFIG **********/
    // 配置DMA1_Channel2 (USART3_TX)
    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart3_tx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    // 使能DMA传输完成中断
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);

    // 配置DMA中断
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*********** USART3_RX DMA CONFIG **********/
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart3_rx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = UART_DMA_BUF_LEN;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // 循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel3, ENABLE);
}
// 处理接收数据
void uart3_process_rx_data(void)
{
    uint16_t current_pos = UART_DMA_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);

    if (current_pos != uart3_rx_write_pos) {
        if (current_pos > uart3_rx_write_pos) {
            // 数据连续
            chry_ringbuffer_write(&g_uartrx, &uart3_rx_buffer[uart3_rx_write_pos],
                                  current_pos - uart3_rx_write_pos);
        } else {
            // 数据回绕
            chry_ringbuffer_write(&g_uartrx, &uart3_rx_buffer[uart3_rx_write_pos],
                                  UART_DMA_BUF_LEN - uart3_rx_write_pos);
            chry_ringbuffer_write(&g_uartrx, uart3_rx_buffer, current_pos);
        }
        uart3_rx_write_pos = current_pos;
    }
}
// 获取接收缓冲区中的数据量
uint16_t uart3_get_rx_data_count(void)
{
    uint16_t current_pos = UART_DMA_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);
    if (current_pos >= uart3_rx_write_pos) {
        return current_pos - uart3_rx_write_pos;
    } else {
        return UART_DMA_BUF_LEN - uart3_rx_write_pos + current_pos;
    }
}
// USART3中断处理函数 - 处理空闲中断
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) {
        // 清除空闲中断标志
        USART_ReceiveData(USART3);

        // 处理接收到的数据
        uart3_process_rx_data();
    }
}
// DMA发送函数
void uart3_dma_send(uint8_t *data, uint16_t len)
{
    if (uart3_dma_tx_busy || len == 0 || len > sizeof(uart3_tx_buffer)) {
        return;
    }

    uart3_dma_tx_busy = 1;

    // 复制数据到发送缓冲区
    memcpy(uart3_tx_buffer, data, len);

    // 禁用DMA通道
    DMA_Cmd(DMA1_Channel2, DISABLE);

    // 设置传输长度
    DMA1_Channel2->CNTR = len;

    // 使能DMA通道
    DMA_Cmd(DMA1_Channel2, ENABLE);

    // 使能USART3的DMA发送请求
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}
// DMA中断处理函数
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2) != RESET) {
        // 清除中断标志
        DMA_ClearITPendingBit(DMA1_IT_TC2);

        // 禁用USART3的DMA发送请求
        USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);

        // 清除忙标志
        uart3_dma_tx_busy = 0;

        chry_dap_usb2uart_uart_send_complete(dma_tx_length);

        // printf("UART3 DMA TX complete!\r\n");
    }
}

void uartx_preinit(void)
{
    uart3_dma_init();
}

/* implment by user */
void chry_dap_usb2uart_uart_send_bydma(uint8_t *data, uint16_t len)
{
    if (uart3_dma_tx_busy) {
        chry_dap_usb2uart_uart_send_complete(0);
        return;
    }

    uart3_dma_send(data, len);
    dma_tx_length = len;
}
/* implment by user */
void chry_dap_usb2uart_uart_config_callback(struct cdc_line_coding *line_coding)
{
    // do nothing
}
