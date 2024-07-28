#include "uart_receive.h"
#define QUEUE_LENGTH 1024
#define ITEM_SIZE sizeof(uint8_t)

void UartRxCallback(void){
	xSemaphoreGiveFromISR(UartRxReady, NULL);
}