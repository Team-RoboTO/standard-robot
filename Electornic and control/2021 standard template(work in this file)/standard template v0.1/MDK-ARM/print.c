#include "main.h"
#include "stdio.h"
int fputc(int ch, FILE *f)
{
 HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
 return ch;
}
