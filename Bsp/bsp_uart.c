#include "bsp_uart.h"
#include "stm32f4xx_it.h"  
#include "usart.h"
#include <string.h>

// 接收数据段
uint8_t UARTBUF[256];
uint8_t len = 0;

/**
 * @brief       UART6初始化函数
 * @param[in]   None
 * @retval      None
 * @note        使能RXNE（接收非空中断）和IDLE（空闲中断）
 */
void uart_init(void){

    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

/**
 * @brief       串口格式化输出函数
 * @param[in]   format: 格式化字符串
 * @param[in]   ...: 可变参数列表
 * @retval      None
 * @warning     内部缓冲区256字节，格式化后的字符串不应超过此长度
 * @note        阻塞式发送，使用HAL_MAX_DELAY
 */ 
void uart_printf(const char *format, ...)
{
    char buffer[256]; 
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart6, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

/**
 * @brief       UART6中断服务函数
 * @param[in]   None
 * @retval      None
 * @note        使用IDLE中断实现帧结束检测
 *              接收的数据存储在UARTBUF缓冲区
 *              接收完成后更新len变量
 */
// void USART6_IRQHandler(void){
    
//     static uint8_t q = 0;
//     if(huart6.Instance->SR & UART_FLAG_RXNE){
//         UARTBUF[q] = huart6.Instance->DR;
//         q++;
//     }
//     else if(huart6.Instance->SR & UART_FLAG_IDLE){
//         huart6.Instance->DR;
//         len = q;
//         q = 0;
//     }
// }

void USART6_IRQHandler(void)
{
    static uint8_t q = 0;
    uint32_t sr = huart6.Instance->SR;
    
    // 处理接收中断
    if(sr & UART_FLAG_RXNE)
    {
        UARTBUF[q] = huart6.Instance->DR;
        q = (q + 1) % 256; // 防止溢出
    }
    
    // 处理空闲中断
    if(sr & UART_FLAG_IDLE)
    {
        volatile uint32_t temp = huart6.Instance->DR; // 清除IDLE标志
        len = q;
        q = 0;
    }
    
    if(sr & (UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE))
    {
        volatile uint32_t temp = huart6.Instance->DR;
        temp = huart6.Instance->SR;
        // 可以在这里添加错误处理代码
    }
}

/**
 * @brief       串口指令解析函数
 * @param[in]   targStr: 目标指令字符串
 * @param[out]  flag: 匹配成功标志，1:匹配成功，0:未匹配
 * @retval      None
 */
void uart_func(const char* targStr,  uint8_t* flag){

    uint8_t targLen = strlen(targStr);
    if(len >= targLen && !strncmp((char*)UARTBUF, targStr, targLen)){
        *flag = 1;
    }
}

/**
 * @brief       接收数据显示函数（调试用）
 * @param[in]   None
 * @retval      None
 */
void uart_receive_show(void){

    uart_printf("Receive_Show: ");
    for(int i = 0; i < len; i++){
            uart_printf("%c", UARTBUF[i]);
    }
    uart_printf("\r\n");
}

