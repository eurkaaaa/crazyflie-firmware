#ifndef __EVENT_H__
#define __EVENT_H__

#define EVENT_NOT_CLEAR_ON_ENTRY 0x0
#define EVENT_CLEAR_ON_ENTRY 0xfffff
#define EVENT_NOT_CLEAR_ON_EXIT 0x0
#define EVENT_CLEAR_ON_EXIT 0xfffff

#define EVENT_DEBUG_ENABLE

typedef enum {
    EVENT_CONGESTION = 1
} EVENT_TYPE;

void eventInit();
TaskHandle_t eventGetTaskHandle();

#endif