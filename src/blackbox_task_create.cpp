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
 * by Nicholas Sherlock (aka thenickdude), which has a GPLv3 license,
 * see https://github.com/thenickdude/blackbox
 */

#include "blackbox_task.h"

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


BlackboxTask* BlackboxTask::create_task(Blackbox& blackbox, MessageQueueBase& message_queue, blackbox_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds) // NOLINT(readability-convert-member-functions-to-static)
{
    task_info_t task_info {};
    return create_task(task_info, blackbox, message_queue, parameter_group, priority, core, task_interval_microseconds);
}

BlackboxTask* BlackboxTask::create_task(task_info_t& task_info, Blackbox& blackbox, MessageQueueBase& message_queue, blackbox_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds) // NOLINT(readability-convert-member-functions-to-static)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static BlackboxTask blackbox_task(task_interval_microseconds, blackbox, message_queue, parameter_group);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t task_parameters { // NOLINT(misc-const-correctness) false positive
        .task = &blackbox_task
    };
#if !defined(BLACKBOX_TASK_STACK_DEPTH_BYTES)
    enum { BLACKBOX_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, BLACKBOX_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, BLACKBOX_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .task_handle = nullptr,
        .name = "BlackboxTask",
        .stack_depth_bytes = BLACKBOX_TASK_STACK_DEPTH_BYTES,
        .stack_buffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .task_interval_microseconds = task_interval_microseconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(strlen(task_info.name) < configMAX_TASK_NAME_LEN && "BlackboxTask: taskname too long");
    assert(task_info.priority < configMAX_PRIORITIES && "BlackboxTask: priority too high");

    static StaticTask_t task_buffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    task_info.task_handle = xTaskCreateStaticPinnedToCore(
        BlackboxTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create BlackboxTask");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.task_handle = xTaskCreateStaticAffinitySet(
        BlackboxTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create BlackboxTask");
#else
    task_info.task_handle = xTaskCreateStatic(
        BlackboxTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer
    );
    assert(task_info.task_handle != nullptr && "Unable to create BlackboxTask");
    // vTaskCoreAffinitySet(task_info.task_handle, task_info.core);
#endif
#else
    (void)task_parameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &blackbox_task;
}
