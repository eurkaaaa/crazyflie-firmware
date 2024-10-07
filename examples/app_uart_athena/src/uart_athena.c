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

struct fly_parm
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
};

// extern SemaphoreHandle_t UartRxReady;
static uint8_t Pos[6];
static uint8_t Pos_new[6];
static TimerHandle_t positionTimer;
static uint8_t j = 1;
static TaskHandle_t appMainTask_Handler;
static setpoint_t setpoint;
static float height = 0.5;

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

void printPara_Uint(int para)
{
    DEBUG_PRINT("%u \t", Pos_new[para]);
}

void printPara_Char(int para)
{
    DEBUG_PRINT("%c", Pos_new[para] + '\0');
    // DEBUG_PRINT("%u \n", para);
}

static void Init()
{
    positionTimer = xTimerCreate("positionTimer", M2T(1000), pdTRUE, (void*)0, Fly_parm_update);
    xTimerStart(positionTimer, M2T(0));
    UartRxReady = xSemaphoreCreateMutex();
    uart1Init(115200);
    // Pos[0] = 1;
    // for(int i=1; i<7; i++)
    // {
    //     Pos[i] = i;
    // }
    // uart1SendData(sizeof(Pos),Pos);
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
    DEBUG_PRINT("uart_receive ...succ\n");
    uint8_t index = 0;
    int type;
    bool flag = 0;
    for(;;)
    {    
      if (xSemaphoreTake(UartRxReady, 0) == pdPASS) 
      {
         while (index < 6 && xQueueReceive(uart1queue, &Pos_new[index], 0) == pdPASS) 
        {
			if(Pos_new[index] != 0)
            {
                vTaskDelay(M2T(100));
                if(!flag)
                {
                    if(Pos_new[index] == 1 || Pos_new[index] == 2)
                    {
                        type = Pos_new[index];
                        flag = 1;
                    }
                }
                else
                {
                    switch (type)
                    {
                    case 1:
                        printPara_Char(index);
                    break;            
                    default:
                        printPara_Uint(index);
                    break;
                    }
                    // DEBUG_PRINT("type = %d \n", type);
                }
			}
            if(type == 1 && Pos_new[index] == '\0')
            {
                flag = 0, index = 0;
                vTaskDelay(M2T(100));
                DEBUG_PRINT("\n");
            }
            index++;
		}
		if(index == 6)
		{
        //ask driver to fly
            index = 0;
		}
      }
      vTaskDelay(M2T(200));
    }
}

void appMain()
{
    //init
    Init();
    xTaskCreate(Uart_Receive, "main_task", TASK_SIZE, NULL, TASK_PRI, &appMainTask_Handler);
    DEBUG_PRINT("main_task ...succ\n");
}