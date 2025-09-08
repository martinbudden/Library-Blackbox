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
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif


BlackboxTask* BlackboxTask::createTask(Blackbox& blackbox, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, blackbox, priority, coreID, taskIntervalMicroSeconds);
}

BlackboxTask* BlackboxTask::createTask(task_info_t& taskInfo, Blackbox& blackbox, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
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
    static std::array <uint8_t, BLACKBOX_TASK_STACK_DEPTH_BYTES> stack;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "BlackboxTask",
        .stackDepth = BLACKBOX_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
        .taskIntervalMicroSeconds = taskIntervalMicroSeconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "BlackboxTask: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "BlackboxTask: priority too high");

    static StaticTask_t taskBuffer;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        BlackboxTask::Task,
        taskInfo.name,
        taskInfo.stackDepth / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    taskInfo.taskHandle = taskHandle;
    assert(taskHandle != nullptr && "Unable to create Blackbox task.");
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &blackboxTask;
}
