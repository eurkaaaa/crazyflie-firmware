#include "uart_receive.h"
#define QUEUE_LENGTH 1024
#define ITEM_SIZE sizeof(uint8_t)

// QueueHandle_t uart1queue;
// STATIC_MEM_QUEUE_ALLOC(uart1queue, QUEUE_LENGTH, sizeof(uint8_t));

// void CreateUartRxQueue(void) {
//     uart1queue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
//     ASSERT(uart1queue != NULL);
// }

void UartRxCallback(void){
	xSemaphoreGiveFromISR(UartRxReady, NULL);
}