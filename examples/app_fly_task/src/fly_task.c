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
 *
 *
 * hello_file_tree.c - App layer application of a simple hello world app with the functionality implemented in
 * files that are located in multiple directories.
 */

#include "FreeRTOS.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "task.h"
#include "log.h"
#include "square_task.h"
#include "triangle_task.h"
#include "eight_shaped_task.h"
#include "semphr.h"
#define DEBUG_MODULE "THEAPP"
#include "debug.h"

#define APP_STACK_SIZE 1000
#define APP_MAIN_PRI 1
TaskHandle_t appMainTask_Handlar;

#define SQUARE_TASK_SIZE 2 * configMINIMAL_STACK_SIZE
#define SQUARE_TASK_PRI 3 // 数字大优先级高
TaskHandle_t square_task_Handlar;

#define TRIANGLE_TASK_SIZE 2 * configMINIMAL_STACK_SIZE
#define TRIANGLE_TASK_PRI 2
TaskHandle_t triangle_task_Handlar;

#define EIGHT_SHAPED_TASK_SIZE 2 * configMINIMAL_STACK_SIZE
#define EIGHT_SHAPED_TASK_PRI 2
TaskHandle_t eight_shaped_task_Handlar;

QueueHandle_t fly_Semaphore;
int f_flag = 0;
void appMainTask(void *param);

void appMain()
{
    DEBUG_PRINT("Waiting for activation ...\n");
    DEBUG_PRINT("appMain Hello!\n");
    vTaskDelay(M2T(3000));
    fly_Semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(fly_Semaphore);

    xTaskCreate(square_task, "square_task", SQUARE_TASK_SIZE, NULL, SQUARE_TASK_PRI, &square_task_Handlar);
    DEBUG_PRINT("square_task ...succ\n");

    // xTaskCreate(triangle_task, "triangel_task", TRIANGLE_TASK_SIZE, NULL, TRIANGLE_TASK_PRI, &triangle_task_Handlar);
    // DEBUG_PRINT("triangle_task ...succ\n");

    // xTaskCreate(eight_shaped_task, "eight_shaped_task", EIGHT_SHAPED_TASK_SIZE, NULL, EIGHT_SHAPED_TASK_PRI, &eight_shaped_task_Handlar);
    // DEBUG_PRINT("eight_shaped_task ...succ\n");
}
