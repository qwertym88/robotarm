#include "pump.h"

uint8_t pumpStatus = 0;

/**
 * @brief switch on pump
 */
uint8_t pump_on(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    pumpStatus = 1;
    return 1;
}
/**/

/**
 * @brief switch off pump
 */
uint8_t pump_off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    pumpStatus = 0;
    return 0;
}

uint8_t pump_status(void)
{
    return pumpStatus;
}
