/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "configblock.h"
#include "system.h"
#include "square_task.h"
#include "semphr.h"
#include "debug.h"
#include "param.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "log.h"
#define DEBUG_MODULE "SQUARE_TASK"
static setpoint_t setpoint;
static float height = 2.0;
extern SemaphoreHandle_t fly_Semaphore;

State state = idle;
Stage stage = onGround;
extern int f_flag;


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
void control_fly_square(int fly_case)
{
    switch (fly_case)
    {
    case stage_one:
        for (int i = 0; i < 100; i++)
        {
            setHoverSetpoint(&setpoint, 0.1, 0, height, 0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_one_square\n");
        break;

    case stage_two:
        for (int i = 0; i < 100; i++)
        {
            setHoverSetpoint(&setpoint, 0, 0.1, height, 0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_two_square\n");
        break;

    case stage_three:
        for (int i = 0; i < 100; i++)
        {
            setHoverSetpoint(&setpoint, -0.1, 0, height, 0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_three_square\n");
        break;

    case stage_four:
        for (int i = 0; i < 100; i++)
        {
            setHoverSetpoint(&setpoint, 0, -0.1, height, 0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_four_square\n");
        break;
    }
}
float get_min(float *var_history, int len_history)
{
    float res = var_history[0];
    for (uint8_t i = 1; i < len_history; i++)
    {
        res = res < var_history[i] ? res : var_history[i];
    }
    return res;
}
float get_max(float *var_history, int len_history)
{
    float res = var_history[0];
    for (uint8_t i = 1; i < len_history; i++)
    {
        res = res > var_history[i] ? res : var_history[i];
    }
    return res;
}
void waitingEstimators()
{
    int len_history = 10;
    float var_x_history[len_history];
    float var_y_history[len_history];
    float var_z_history[len_history];
    for (uint8_t i = 0; i < len_history; i++)
    {
        var_x_history[i] = 1000.0;
        var_y_history[i] = 1000.0;
        var_z_history[i] = 1000.0;
    }
    float threshold = 0.001;
    uint8_t i = 0;
    while (true)
    {
        float varPX = logGetVarId("kalman", "varPX");
        float varPY = logGetVarId("kalman", "varPY");
        float varPZ = logGetVarId("kalman", "varPZ");
        var_x_history[i] = varPX;
        var_y_history[i] = varPY;
        var_z_history[i] = varPZ;

        float min_x = get_min(var_x_history, len_history);
        float max_x = get_max(var_x_history, len_history);
        float min_y = get_min(var_y_history, len_history);
        float max_y = get_max(var_y_history, len_history);
        float min_z = get_min(var_z_history, len_history);
        float max_z = get_max(var_z_history, len_history);

        if (((max_x - min_x) < threshold) &&
            ((max_y - min_y) < threshold) &&
            ((max_z - min_z) < threshold))
        {
            break;
        }
        i = (i + 1) % len_history;
    }
}
void square_task(void *param)
{
    systemWaitStart();
    DEBUG_PRINT("Estimatorsing\n");
    waitingEstimators();
    DEBUG_PRINT("Estimators succ\n");
    BaseType_t result = pdFALSE;
    result = xSemaphoreTake(fly_Semaphore, portMAX_DELAY);
    while (1)
    {
        if (result == pdFALSE)
        {
            result = xSemaphoreTake(fly_Semaphore, portMAX_DELAY);
        }
        else
        {
            DEBUG_PRINT("square_task having semaphore\n");
            switch (state)
            {
            case flying:

                control_fly_square(stage);
                if (stage == stage_four)
                {

                    if (f_flag == 0)
                    {
                        state = landing;
                    }
                    result = pdFALSE;
                    stage = stage_one;
                    xSemaphoreGive(fly_Semaphore);
                    vTaskDelay(M2T(10));
                }
                else
                {
                    stage++;
                }
                DEBUG_PRINT("%ld\n", result);

                break;

            case landing:
                DEBUG_PRINT("landing\n");
                land();
                DEBUG_PRINT("land succ\n");
                state = idle;
                stage = onGround;

                vTaskDelay(M2T(1000));
                break;

            case idle:
                if (stage == onGround && f_flag == 1)
                {
                    DEBUG_PRINT("take off is runing...\n");
                    take_off();
                    stage++;
                    state = flying;
                }
                vTaskDelay(M2T(10));
                break;
            }
        }
    }
}

PARAM_GROUP_START(f_task)
PARAM_ADD(PARAM_UINT8, f_flag, &f_flag)
PARAM_GROUP_STOP(f_task)