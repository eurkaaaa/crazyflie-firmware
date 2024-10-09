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
#include "stream_buffer.h"

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
// static uint8_t Pos[16];
static uint8_t Pos_new[16];
static uint8_t state[1];
static TimerHandle_t positionTimer;
static TaskHandle_t appMainTask_Handler;
static setpoint_t setpoint;
static float height = 1.0;
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

void para_init()
{
    Para[0] = 1.0;
    Para[1] = 2.0;
    Para[2] = 5.0;
    Para[3] = 8.0;
    uint8_t *Pos = (uint8_t *)Para;
    for(int i=0;i<16;i++)
    {
        DEBUG_PRINT("%d \t", *(Pos+i));
    }
    DEBUG_PRINT("\n");
    uart2SendData(16, Pos);
    // memcpy(Pos, (uint8_t *)Para, 16);
    // Pos[16] = 0;
    // uart2SendData(17, Pos);
}

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
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = yawrate;
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.z = modeAbs;
    setpoint->position.z = z;
    setpoint->velocity.x = vx;
    setpoint->velocity.y = vy;
   // setpoint->velocity.z = vz;
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
    DEBUG_PRINT("uart_receive ...succ\n");
    uint8_t index = 0;
    for(;;)
    {    
      if (xSemaphoreTake(UartRxReady, 0) == pdPASS) 
      {
        xStreamBufferSetTriggerLevel(rxStream, 1);
        while (index < 6 && xStreamBufferReceive(rxStream, &Pos_new[index], 1, portMAX_DELAY) == 1) 
        {
            index++;
            vTaskDelay(M2T(10));
		}
		if(index == 6)
		{
            // xSemaphoreGive(ParaReady);
            index = 0;
		}
      }
      vTaskDelay(M2T(10));
    }
}

static void Fly()
{
    float para[4];
    bool flag = 0;
    memcpy(para, (float *)Pos_new, 16);
    for(int i=0;i<4;i++)
    {
        if(para[i] != 0)
        {
            flag = 1;
        }
    }
    if(flag == 0)
    {
        land();
        return;
    }
    for(int i=0;i < 100;i++)
    {
        setHoverSetpoint(&setpoint, para[0], para[1], para[2], para[3]);
        vTaskDelay(M2T(10));
    }
   // vTaskDelay(10000);
    // for(int i=0;i<4;i++)
    // {
    //     DEBUG_PRINT("%f \t", para[i]);
    // }
    // DEBUG_PRINT("\n");
}

void appMain()
{
    // vTaskDelay(5000);
    UartRxReady = xSemaphoreCreateMutex();
    ParaReady = xSemaphoreCreateMutex();
    uart2Init(115200);
    vTaskDelay(M2T(5000));
    state[0] = 0;
    while(1)
    {
        // para_init();
        if(state[0]<6)
        {
            uart2SendData(1, state);
            DEBUG_PRINT("send\n");
            uart2GetData(16, Pos_new);
            // for(int i=0;i<16;i++)
            // {
            //     DEBUG_PRINT("%d \t",Pos_new[i]);
            // }
            // DEBUG_PRINT("\n");
            Fly();
            DEBUG_PRINT("rece \n");
            state[0]++;
            DEBUG_PRINT("%d", state[0]);
        }
        vTaskDelay(M2T(10));
    }
}