#include "LED.h"
#include "main.h"
void LED_ON(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
}


void LED_OFF(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
}


void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}
