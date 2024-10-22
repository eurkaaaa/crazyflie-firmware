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
// static uint8_t Pos[17];
// static uint8_t Pos_new[17];
static uint8_t Pos[36];
static uint8_t Pos_new[36];
static uint8_t state[1];
static TimerHandle_t positionTimer;
static TaskHandle_t appMainTask_Handler;
static setpoint_t setpoint;
static float height = 1.0;
static float Para[9];
static float para_new[4];
static float padX;
static float padY;
static float padZ;
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdStateEstimateYaw;
static logVarId_t logIdStateEstimatePitch;
static logVarId_t logIdStateEstimateRoll;
static float getX() { return logGetFloat(logIdStateEstimateX); }
static float getY() { return logGetFloat(logIdStateEstimateY); }
static float getZ() { return logGetFloat(logIdStateEstimateZ); }
static float getYaw() { return logGetFloat(logIdStateEstimateYaw); }
static float getPitch() { return logGetFloat(logIdStateEstimatePitch); }
static float getRoll() { return logGetFloat(logIdStateEstimateRoll); }

void para_init()
{
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdStateEstimateYaw = logGetVarId("stateEstimate", "yaw");
    logIdStateEstimatePitch = logGetVarId("stateEstimate", "pitch");
    logIdStateEstimateRoll = logGetVarId("stateEstimate", "roll");

    Para[0] = getX();
    Para[1] = getY();
    Para[2] = getZ();
    Para[3] = getYaw();
    Para[4] = getPitch();
    Para[5] = getRoll();

    memcpy(Pos, (uint8_t *)Para, 24);
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


static void Fly()
{
    float para[4];
    bool flag = 0;
    memcpy(para, (float *)Pos_new, 16);
    // for(int i=0;i<4;i++)
    // {
    //     if(para[i] != 0)
    //     {
    //         flag = 1;
    //     }
    // }
    // if(flag == 0)
    // {
    //     land();
    //     return;
    // }
    // for(int i=0;i < 100;i++)
    // {
    //     setHoverSetpoint(&setpoint, para[0], para[1], para[2], para[3]);
    //     vTaskDelay(M2T(10));
    // }
   // vTaskDelay(10000);
    // for(int i=0;i<4;i++)
    // {
    //     DEBUG_PRINT("%f \t", para[i]);
    // }
    // DEBUG_PRINT("\n");
}

void para_get()
{
    padX = para_new[0];
    padY = para_new[1];
    padZ = para_new[2];
}

void appMain()
{
    // vTaskDelay(5000);
    UartRxReady = xSemaphoreCreateMutex();
    ParaReady = xSemaphoreCreateMutex();
    uart2Init(115200);
    vTaskDelay(M2T(10000));
    Pos[24] = 0;
    Pos[25] = 0;
    while(1)
    {
        para_init();
        DEBUG_PRINT("crazyflie = %d \n",Pos[24]);
        uart2SendData(26, Pos);
        // for(int i=0;i<26;i++)
        // {
        //     DEBUG_PRINT("%d \t", Pos[i]);
        // }
        // DEBUG_PRINT("send \n");
        vTaskDelay(100);
        uart2GetData(17, Pos_new);
        memcpy(para_new, (float *)Pos_new, 16); 
        para_get();
        DEBUG_PRINT("X = %d \n",padX);
        DEBUG_PRINT("Y = %d \n",padY);
        DEBUG_PRINT("Z = %d \n",padZ);
        for(int i=0;i<4;i++)
        {
            DEBUG_PRINT("%f \t", *((float *)(Pos_new) + i));
        }
        DEBUG_PRINT("\n");
        DEBUG_PRINT("athena = %d \n",Pos_new[16]);
        switch (Pos_new[16])
        {  
        case 1:
            // crtpCommanderHighLevelTakeoff(padZ, 1.0f);
            Pos[24] = 1;
            break;
        case 2: 
            // crtpCommanderHighLevelGoTo(padX, padY, padZ, 0.0f, 0.1f, 0);
            Pos[24] = 1;
            break;

        case 3:
            // crtpCommanderHighLevelLand(padZ, 1.0f);
            Pos[24] = 0;
            break;               
        default:
            break;
        }
        // for(int i=0;i<17;i++)
        // {
        //     DEBUG_PRINT("%d \t", Pos_new[i]);
        // }
        // DEBUG_PRINT("receive \n");
        vTaskDelay(M2T(10));
    }
}

PARAM_GROUP_START(f_t)
PARAM_ADD(PARAM_UINT8, flag, &Pos[25])
PARAM_GROUP_STOP(f_t)