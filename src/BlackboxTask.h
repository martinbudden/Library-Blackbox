#pragma once

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

#include <TaskBase.h>
class Blackbox;
class BlackboxCallbacksBase;
class BlackboxMessageQueueBase;


class BlackboxTask : public TaskBase {
public:
    BlackboxTask(uint32_t taskIntervalMicroSeconds, Blackbox& blackbox);
public:
    static BlackboxTask* createTask(task_info_t& taskInfo, Blackbox& blackbox, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static BlackboxTask* createTask(Blackbox& blackbox, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    BlackboxMessageQueueBase& getMessageQueue() { return _messageQueue; }
private:
    // class is not copyable or moveable
    BlackboxTask(const BlackboxTask&) = delete;
    BlackboxTask& operator=(const BlackboxTask&) = delete;
    BlackboxTask(BlackboxTask&&) = delete;
    BlackboxTask& operator=(BlackboxTask&&) = delete;
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    Blackbox& _blackbox;
    BlackboxMessageQueueBase& _messageQueue;
};
