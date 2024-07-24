#ifndef UART_RECEIVE_H
#define UART_RECEIVE_H
#include <string.h>

#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "static_mem.h"

extern SemaphoreHandle_t UartRxReady;
extern QueueHandle_t uart1queue;

// void CreateUartRxQueue(void);
void UartRxCallback(void);

#endif
