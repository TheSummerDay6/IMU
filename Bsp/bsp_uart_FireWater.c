#include "bsp_uart_FireWater.h"
#include "bsp_uart.h"  
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "usart.h"

/**
  * @brief  FireWater协议Printf函数（自动添加换行符结尾）
  * @param  format: 格式字符串
  * @param  ...:    可变参数
  * @retval None
  * @note   自动在末尾添加\\n，符合FireWater协议
  */
void FireWater_printf(const char* format, ...)
{
    char buffer[256];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    uart_printf("%s\n", buffer);
}

/**
  * @brief  发送图片前导帧
  * @param  img_id:     图片通道ID
  * @param  img_size:   图片数据大小（字节）
  * @param  img_width:  图片宽度
  * @param  img_height: 图片高度
  * @param  img_format: 图片格式
  * @retval None
  * @note   格式："image:id,size,width,height,format\\n"
  */
void FireWater_SendImageHeader(uint32_t img_id, uint32_t img_size, 
                               uint32_t img_width, uint32_t img_height, 
                               uint32_t img_format)
{
    uart_printf("image:%lu,%lu,%lu,%lu,%lu\n", 
                img_id, img_size, img_width, img_height, img_format);
}

/**
  * @brief  发送图片数据
  * @param  img_data: 图片数据指针
  * @param  img_size: 图片数据大小
  * @retval None
  * @warning 必须在前导帧后立即发送！
  */
void FireWater_SendImageData(uint8_t* img_data, uint32_t img_size)
{
    if(img_data == NULL || img_size == 0) return;
    
    // 直接发送二进制图片数据
    HAL_UART_Transmit(&huart6, img_data, img_size, HAL_MAX_DELAY);
}

/*
    // 示例：发送传感器数据
    float sensor_data[4] = {1.23f, 4.56f, 7.89f, 0.12f};

    // 方法1：不带标签（VOFA+中显示为纯数据）
    FireWater_SendData(sensor_data, 4);

    // 方法2：带标签（VOFA+中显示为"sensor:1.23,4.56,7.89,0.12"）
    FireWater_SendDataWithTag("sensor", sensor_data, 4);

    // 方法3：发送调试信息
    FireWater_SendString("System started successfully");
    FireWater_SendString("Temperature: 25.6°C");  // 会自动添加\n
*/