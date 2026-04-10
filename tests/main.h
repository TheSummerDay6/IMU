/**
 * @file main.h (test stub)
 * @brief Minimal stub for main.h to enable host-side unit testing.
 */
#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>

/* Stub GPIO types */
typedef uint32_t GPIO_TypeDef;

/* Stub pin defines */
#define GPIO_PIN_0  0
#define GPIO_PIN_4  4
#define GPIO_PIN_5  5
#define GPIO_PIN_6  6
#define GPIO_PIN_7  7
#define GPIO_PIN_8  8
#define GPIO_PIN_12 12

static inline void Error_Handler(void) {}

#endif /* __MAIN_H */
