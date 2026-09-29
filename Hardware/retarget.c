#include "retarget.h"
#include "usart.h"
#include <stdio.h>


int fputc(int ch, FILE *f)
{

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)&ch,
                      1,
                      100);

    return ch;

}


