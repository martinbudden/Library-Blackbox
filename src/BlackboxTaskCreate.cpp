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

#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif
#endif

#include <array>
#include <cstring>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif


BlackboxTask* BlackboxTask::createTask(task_info_t& taskInfo, Blackbox& blackbox, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static BlackboxTask blackboxTask(taskIntervalMicroSeconds, blackbox);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &blackboxTask
    };
#if !defined(BLACKBOX_TASK_STACK_DEPTH_BYTES)
    enum { BLACKBOX_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
    static std::array <StackType_t, BLACKBOX_TASK_STACK_DEPTH_BYTES> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "BlackboxTask",
        .stackDepth = BLACKBOX_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "BlackboxTask: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "BlackboxTask: priority too high");

    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        BlackboxTask::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    taskInfo.taskHandle = taskHandle;
    assert(taskHandle != nullptr && "Unable to create Blackbox task.");

#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** BlackboxTask,      core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    Serial.print(&buf[0]);
#endif
#endif
#else
    (void)taskInfo;
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &blackboxTask;
}

BlackboxTask* BlackboxTask::createTask(Blackbox& blackbox, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, blackbox, priority, coreID, taskIntervalMicroSeconds);
}
