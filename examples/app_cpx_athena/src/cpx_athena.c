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
#include "crtp_commander_high_level.h"

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
static uint8_t Pos[26];
// static uint8_t Pos_new[17];
// static uint8_t Pos[16];
static uint8_t Pos_new[17];
// static uint8_t state[1];
static TimerHandle_t positionTimer;
static TaskHandle_t appMainTask_Handler;
static setpoint_t setpoint;
static float height = 0.5f;
static float Para[6];
static float para_new[4];

static void Fly_parm_update()
{
    // Get the logging data
    logVarId_t idYaw = logGetVarId("stateEstimate", "yaw");
    logVarId_t idPitch = logGetVarId("stateEstimate", "pitch");
    logVarId_t idRoll = logGetVarId("stateEstimate", "roll");
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idZ = logGetVarId("stateEstimate", "z");

    Para[0] = logGetFloat(idX);
    Para[1] = logGetFloat(idY);
    Para[2] = logGetFloat(idZ);
    Para[3] = logGetFloat(idYaw);
    Para[4] = logGetFloat(idPitch);
    Para[5] = logGetFloat(idRoll);

    memcpy(Para, (uint8_t *)Pos, 24); 
}


// static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
// {
//     setpoint->mode.yaw = modeVelocity;
//     setpoint->attitudeRate.yaw = yawrate;
//     setpoint->mode.x = modeVelocity;
//     setpoint->mode.y = modeVelocity;
//     setpoint->mode.z = modeAbs;
//     setpoint->position.z = z;
//     setpoint->velocity.x = vx;
//     setpoint->velocity.y = vy;
//    // setpoint->velocity.z = vz;
//     setpoint->velocity_body = true;
//     commanderSetSetpoint(setpoint, 3);
// }

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
    uart2Init(115200);
    Pos_new[16] = 0;
    Pos[25] = 0;
    vTaskDelay(M2T(5000));
    while(1)
    {
        Fly_parm_update();
        for(int i=0;i<26;i++)
        {
            DEBUG_PRINT("%d \t",Pos[i]);
        }
        DEBUG_PRINT("\n");
        uart2SendData(26, Pos);
        DEBUG_PRINT("send\n");
        uart2GetData(17, Pos_new);
        for(int i=0;i<17;i++)
        {
            DEBUG_PRINT("%d \t",Pos_new[i]);
        }
        DEBUG_PRINT("\n");
        DEBUG_PRINT("rece \n");
        switch (Pos_new[16])
        {
        case 0:
            memcpy(para_new, (float *)Pos_new, 16); 
            crtpCommanderHighLevelTakeoff(para_new[2], 1.0f);
            Pos[24] = 1;
            break;
        
        case 1:
            memcpy(para_new, (float *)Pos_new, 16); 
            crtpCommanderHighLevelGoTo(para_new[0], para_new[1], para_new[2], para_new[3], 0.1f, 0);
            Pos[24] = 1;
            break;

        case 2:
            memcpy(para_new, (float *)Pos_new, 16); 
            crtpCommanderHighLevelLand(para_new[2], 1.0f);
            Pos[24] = 0;
            break;
        default:
            break;
        }
        // for(int i=0;i<16;i++)
        // {
        //     DEBUG_PRINT("%d \t",Pos_new[i]);
        // }
        // DEBUG_PRINT("\n");
        vTaskDelay(M2T(10));
    }
}

PARAM_GROUP_START(f_t)
PARAM_ADD(PARAM_UINT8, flag, &Pos[25])
PARAM_GROUP_STOP(f_t)