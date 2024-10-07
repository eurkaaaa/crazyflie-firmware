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
extern xQueueHandle uart1queue;
extern xQueueHandle uart2queue;

void UartRxCallback(void);

#endif
