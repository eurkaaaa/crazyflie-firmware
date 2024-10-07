#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "uart_receive.h"
#include "uart2.h"
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

struct fly_parm
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
};

SemaphoreHandle_t ParaReady;
// static uint8_t Pos[17];
// static uint8_t Pos_new[17];
static uint8_t Pos_new[16];
static TimerHandle_t positionTimer;
static TaskHandle_t appMainTask_Handler;
static setpoint_t setpoint;
static float height = 0.5;
static float Para[4];

// void Fly_parm_update()
// {
//     // Get the logging data
//     logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");
//     logVarId_t idPitch = logGetVarId("stateEstimate", "pitch");
//     logVarId_t idRoll = logGetVarId("stateEstimate", "roll");
//     logVarId_t idX = logGetVarId("stateEstimate", "x");
//     logVarId_t idY = logGetVarId("stateEstimate", "y");
//     logVarId_t idZ = logGetVarId("stateEstimate", "z");

//     Pos[0] = logGetFloat(idYaw);
//     Pos[1] = logGetFloat(idPitch);
//     Pos[2] = logGetFloat(idRoll);
//     Pos[3] = logGetFloat(idX);
//     Pos[4] = logGetFloat(idY);
//     Pos[5] = logGetFloat(idZ);

//     uart2SendData(sizeof(Pos),Pos);
// }

void para_update()
{
    Para[0] = setpoint.velocity.x;
    Para[1] = setpoint.velocity.y;
    Para[2] = setpoint.position.z;
    Para[3] = setpoint.attitudeRate.yaw;
    uint8_t *Pos = (uint8_t *)Para;
    uart2SendData(16, Pos);
    // memcpy(Pos, (uint8_t *)Para, 16);
    // Pos[16] = 0;
    // uart2SendData(17, Pos);
}


static void Init()
{
    // positionTimer = xTimerCreate("positionTimer", M2T(200), pdTRUE, (void*)0, parm_update);
    // xTimerStart(positionTimer, M2T(0));
    UartRxReady = xSemaphoreCreateMutex();
    ParaReady = xSemaphoreCreateMutex();
    uart2Init(115200);
}

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
    setpoint->mode.z = modeAbs;
    setpoint->position.z = z;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = yawrate;
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = vx;
    setpoint->velocity.y = vy;
    setpoint->velocity_body = true;
    commanderSetSetpoint(setpoint, 3);
}

void take_off()
{
    for (int i = 0; i < 100; i++)
    {
        setHoverSetpoint(&setpoint, 0, 0, height, 0);
        vTaskDelay(M2T(10));
    }
}
void land()
{
    int i = 0;
    float per_land = 0.05;
    while (height - i * per_land >= 0.05f)
    {
        i++;
        setHoverSetpoint(&setpoint, 0, 0, height - (float)i * per_land, 0);
        vTaskDelay(M2T(10));
    }
}

static void Uart_Receive()
{
    // DEBUG_PRINT("uart_receive ...succ\n");
    uint8_t index = 0;
    for(;;)
    {    
      if (xSemaphoreTake(UartRxReady, 0) == pdPASS) 
      {
        while (index < 16 && xQueueReceive(uart2queue, &Pos_new[index], 0) == pdPASS) 
        {
            vTaskDelay(M2T(1));
            index++;
		}
		if(index == 16)
		{
            xSemaphoreGive(ParaReady);
            index = 0;
		}
      }
      vTaskDelay(M2T(10));
    }
}

static void Fly()
{
    float para[4];
    memcpy(para, (float *)Pos_new, 16);
    // uint8_t t = Pos_new[17];
    for(int i=0;i < 100;i++)
    {
        setHoverSetpoint(&setpoint, para[0], para[1], para[2], para[3]);
        vTaskDelay(M2T(1));
    }
}

void appMain()
{
    UartRxReady = xSemaphoreCreateMutex();
    ParaReady = xSemaphoreCreateMutex();
    positionTimer = xTimerCreate("positionTimer", M2T(200), pdTRUE, (void*)0, para_update);
    xTimerStart(positionTimer, M2T(0));    
    // xTaskCreate(Uart_Receive, "main_task", TASK_SIZE, NULL, TASK_PRI, &appMainTask_Handler);
    // DEBUG_PRINT("main_task ...succ\n");
    uart2Init(115200);
    vTaskDelay(M2T(10000));
    //plan 1
    while(1)
    {
        if (xSemaphoreTake(ParaReady, 0) == pdPASS) 
        {
            Fly();
        }
        vTaskDelay(M2T(100));
    }
    //plan 2
    // while(1)
    // {
    //     para_update();
    //     uart2GetData(16, Pos_new);
    //     Fly();
    // }

}