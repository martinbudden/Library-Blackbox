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

#include "blackbox.h"
#include "blackbox_task.h"

#include <cassert>
#include <message_queue_base.h>
#include <time_microseconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


BlackboxTask::BlackboxTask(uint32_t task_interval_microseconds, Blackbox& blackbox, MessageQueueBase& message_queue) :
    TaskBase(task_interval_microseconds),
    _task_interval_milliseconds(task_interval_microseconds/1000), // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _blackbox(blackbox),
    _message_queue(message_queue)
{
}

/*!
loop() function for when not using FREERTOS
*/
void BlackboxTask::loop()
{
    const uint32_t time_microseconds = time_us();
    _time_microseconds_delta = time_microseconds - _time_microseconds_previous;

    if (_time_microseconds_delta >= _task_interval_microseconds) { // if _taskIntervalMicroSeconds has passed, then run the update
        _time_microseconds_previous = time_microseconds;
        _blackbox.update_log(time_microseconds);
    }
}

/*!
Task function for the MSP. Sets up and runs the task loop() function.
*/
[[noreturn]] void BlackboxTask::task() // NOLINT(readability-convert-member-functions-to-static)
{
#if defined(FRAMEWORK_USE_FREERTOS)
    if (_task_interval_microseconds == 0) {
        uint32_t time_microseconds {};
        while (true) {
            _message_queue.WAIT(time_microseconds); // wait until there is AHRS data.
            _blackbox.update_log(time_microseconds);
        }
    } else {
        // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
        const uint32_t task_interval_ticks = pdMS_TO_TICKS(_task_interval_microseconds / 1000);
        assert(task_interval_ticks > 0 && "Blackbox task_interval_ticks is zero.");

        _previous_wake_time_ticks = xTaskGetTickCount();
        while (true) {
            // delay until the end of the next task_interval_ticks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
#endif
            const uint32_t time_microseconds = time_us();
            _blackbox.update_log(time_microseconds);
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for BlackboxTask::task_static with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void BlackboxTask::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<BlackboxTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
