/*
 * This file is part of the Blackbox library.
 *
 * The Blackbox library is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * The Blackbox library is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * The Blackbox library is a port (modification) of the Blackbox implementation
 * in Betaflight (which itself was a port of the Cleanflight implementation).
 *
 * The original Betaflight copyright notice is included below, as per the GNU GPL
 * "keep intact all notices” requirement.
 */

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "BlackboxTask.h"

#include <array>
#include <cassert>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


BlackboxTask* BlackboxTask::createTask(Blackbox& blackbox, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, blackbox, priority, core, taskIntervalMicroSeconds);
}

BlackboxTask* BlackboxTask::createTask(task_info_t& taskInfo, Blackbox& blackbox, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static BlackboxTask blackboxTask(taskIntervalMicroSeconds, blackbox);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &blackboxTask
    };
#if !defined(BLACKBOX_TASK_STACK_DEPTH_BYTES)
    enum { BLACKBOX_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, BLACKBOX_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, BLACKBOX_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "BlackboxTask",
        .stackDepthBytes = BLACKBOX_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .taskIntervalMicroSeconds = taskIntervalMicroSeconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "BlackboxTask: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "BlackboxTask: priority too high");

    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        BlackboxTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BlackboxTask");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    taskInfo.taskHandle = xTaskCreateStaticAffinitySet(
        BlackboxTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BlackboxTask");
#else
    taskInfo.taskHandle = xTaskCreateStatic(
        BlackboxTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BlackboxTask");
    // vTaskCoreAffinitySet(taskInfo.taskHandle, taskInfo.core);
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &blackboxTask;
}
