#include "FreeRTOS.h"
#include "adhocdeck.h"
#include "event.h"
#include "debug.h"

#ifndef EVENT_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

static TaskHandle_t uwbEventTaskHandle = 0;
static uint32_t r_event = 0;
static uint32_t last_event = 0;

static void uwbEventTask(void *parameters) {
    systemWaitStart();

    BaseType_t xReturn = pdTRUE;
    while (true) {
        xReturn = xTaskNotifyWait(EVENT_NOT_CLEAR_ON_ENTRY, EVENT_CLEAR_ON_EXIT,
              &r_event, portMAX_DELAY);
        if(pdTRUE == xReturn) {
            last_event |= r_event;
            /* Handle some event */
            DEBUG_PRINT("Event handle: %u\n", r_event);
            setTxConfigPower(0x00);
        }
    }
}

TaskHandle_t eventGetTaskHandle() {
    return uwbEventTaskHandle;
}

void eventInit() {
    xTaskCreate(uwbEventTask, ADHOC_DECK_EVENT_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbEventTaskHandle);
}