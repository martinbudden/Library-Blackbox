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

#include "Blackbox.h"
#include "BlackboxMessageQueueBase.h"
#include "BlackboxTask.h"

#include <TimeMicroseconds.h>
#include <cassert>

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


BlackboxTask::BlackboxTask(uint32_t taskIntervalMicroseconds, Blackbox& blackbox) :
    TaskBase(taskIntervalMicroseconds),
    _taskIntervalMilliseconds(taskIntervalMicroseconds/1000), // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _blackbox(blackbox),
    _messageQueue(blackbox.getMessageQueue())
{
}

/*!
loop() function for when not using FREERTOS
*/
void BlackboxTask::loop()
{
    const uint32_t timeMicroseconds = timeUs();
    _timeMicrosecondsDelta = timeMicroseconds - _timeMicrosecondsPrevious;

    if (_timeMicrosecondsDelta >= _taskIntervalMicroseconds) { // if _taskIntervalMicroSeconds has passed, then run the update
        _timeMicrosecondsPrevious = timeMicroseconds;
        _blackbox.update(timeMicroseconds);
    }
}

/*!
Task function for the MSP. Sets up and runs the task loop() function.
*/
[[noreturn]] void BlackboxTask::task() // NOLINT(readability-convert-member-functions-to-static)
{
#if defined(FRAMEWORK_USE_FREERTOS)
    if (_taskIntervalMicroseconds == 0) {
        uint32_t timeMicroseconds {};
        while (true) {
            _messageQueue.WAIT_IF_EMPTY(timeMicroseconds); // wait until there is IMU data.
            _blackbox.update(timeMicroseconds);
        }
    } else {
        // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
        const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
        assert(taskIntervalTicks > 0 && "Blackbox taskIntervalTicks is zero.");

        _previousWakeTimeTicks = xTaskGetTickCount();
        while (true) {
            // delay until the end of the next taskIntervalTicks
            vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

            const uint32_t timeMicroseconds = timeUs();
            _blackbox.update(timeMicroseconds);
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for BlackboxTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void BlackboxTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<BlackboxTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
