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