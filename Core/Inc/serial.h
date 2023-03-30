#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "main.h"
#include "gpio.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

enum LOG_LEVEL
{
    DEBUG_LEVEL,
    INFO_LEVEL,
    RELEASE_LEVEL
};
#define CURRENT_LEVEL INFO_LEVEL

// PA4控制着串口的收发
#define USART_OUT HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
// PA4控制着串口的收发
#define USART_IN HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
// rs485专用，速度挺慢的
#define m_printf(format, ...)        \
    do                               \
    {                                \
        USART_OUT;                   \
        printf(format, __VA_ARGS__); \
        HAL_Delay(50);               \
        USART_IN;                    \
    } while (0)

int readStr(uint8_t *buf);
uint8_t UART_available(void);

void print_float(double value);

void print_release(char *str, double a, double b, double c, char *end);
void print_info(char *str, double a, double b, double c, char *end);
void print_debug(char *str, double a, double b, double c, char *end);

#endif
