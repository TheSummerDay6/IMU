/**
  * @file     bsp_uart_firewater.h
  * @brief    VOFA+ FireWater协议接口
  * @author   再睡一夏
  * @date     2026-01-26
  * @version  1.0
  * @note     FireWater协议：文本格式，以换行符\\n结束一帧
  */

#ifndef __BSP_UART_FIREWATER_H
#define __BSP_UART_FIREWATER_H

#include <stdint.h>
#include <stdio.h>

/* FireWater协议传输函数 */
void FireWater_printf(const char* format, ...);

void FireWater_SendImageHeader(uint32_t img_id, uint32_t img_size, 
                               uint32_t img_width, uint32_t img_height, 
                               uint32_t img_format);
void FireWater_SendImageData(uint8_t* img_data, uint32_t img_size);

#endif /* __BSP_UART_FIREWATER_H */