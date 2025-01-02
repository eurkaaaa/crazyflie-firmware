#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "uart_receive.h"
#include "uart1.h"
#include "estimator_kalman.h"
#include "semphr.h"
#include "uart_syslink.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "timers.h"

#include "debug.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "UARTATHENA"

#define BUFFERSIZE 128
#define TASK_SIZE 2 * configMINIMAL_STACK_SIZE
#define TASK_PRI 1 // 数字大优先级高

// extern SemaphoreHandle_t UartRxReady;
static uint8_t buffer[64];
static TimerHandle_t positionTimer;
static TaskHandle_t appMainTask_Handler;

static void Init()
{
    UartRxReady = xSemaphoreCreateMutex();
    uart1Init(115200);
}

static void Uart_Receive()
{
    DEBUG_PRINT("uart_receive ...succ\n");
    uint8_t index = 0;
    for(;;)
    {    
        while (xQueueReceive(uart1queue, &buffer[index], 0) == pdPASS) 
        {
            index++;
		    if(index == 64 || buffer[index - 1] == '\n')
		    {
                consolePuts((const char *)buffer);
                index = 0;
		    }
            vTaskDelay(M2T(10));
		}
       vTaskDelay(M2T(200));
    }
}

void appMain()
{
    //init
    vTaskDelay(M2T(5000));
    Init();
    xTaskCreate(Uart_Receive, "main_task", TASK_SIZE, NULL, TASK_PRI, &appMainTask_Handler);
    DEBUG_PRINT("main_task ...succ\n");
}