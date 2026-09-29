#include "usart_app.h"
#include "main.h"
#include "usart.h"
#include <string.h>


void USART_Printf(char *str)
{

HAL_UART_Transmit(&huart1,
                  (uint8_t *)str,
                  strlen(str),
                  100);

}
