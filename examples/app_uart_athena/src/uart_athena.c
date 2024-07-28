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

#include "timers.h"

#include "debug.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "UARTATHENA"

#define BUFFERSIZE 128

struct fly_parm
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
};

// xQueueHandle uart1queue;
SemaphoreHandle_t UartRxReady = NULL;
static uint8_t Pos[6];
static uint8_t Pos_new[6];
static TimerHandle_t positionTimer;
static uint8_t j = 1;

void Fly_parm_update()
{
    // Get the logging data
    logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");
    logVarId_t idPitch = logGetVarId("stateEstimate", "pitch");
    logVarId_t idRoll = logGetVarId("stateEstimate", "roll");
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idZ = logGetVarId("stateEstimate", "z");

    Pos[0] = logGetFloat(idYaw);
    Pos[1] = logGetFloat(idPitch);
    Pos[2] = logGetFloat(idRoll);
    Pos[3] = logGetFloat(idX);
    Pos[4] = logGetFloat(idY);
    Pos[5] = logGetFloat(idZ);

    uart1SendData(sizeof(Pos),Pos);
}

void para_update()
{
    j+=6;
    for(int i=0; i<6; i++)
    {
        Pos[i] = i+j;
    }
    uart1SendData(sizeof(Pos),Pos);
}

void printPara(int para)
{
    DEBUG_PRINT("%u \t", Pos_new[para]);
}

static void Init()
{
    positionTimer = xTimerCreate("positionTimer", M2T(1000), pdTRUE, (void*)0, para_update);
    xTimerStart(positionTimer, M2T(0));
    UartRxReady = xSemaphoreCreateMutex();
    uart1Init(115200);
    for(int i=0; i<6; i++)
    {
        Pos[i] = i+1;
    }
    uart1SendData(sizeof(Pos),Pos);
}

void appMain()
{
    //init
    Init();
    uint8_t index = 0;
    for(;;)
    {    
      if (xSemaphoreTake(UartRxReady, 0) == pdPASS) 
      {
        while (index < 6 && xQueueReceive(uart1queue, &Pos_new[index], 0) == pdPASS) 
        {
			if(Pos_new[index] != 0)
            {
                vTaskDelay(M2T(100));
                printPara(index);
			    index++;
			}
		}
        DEBUG_PRINT("\n");
		if(index ==6)
		{
        //ask driver to fly 
            index=0;
		}
      }
      vTaskDelay(M2T(200));
	}
}