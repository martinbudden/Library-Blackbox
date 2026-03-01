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
 * by Nicholas Sherlock (aka thenickdude), which has a GPLv3 license,
 * see https://github.com/thenickdude/blackbox
 */

#include <task_base.h>

class Blackbox;
class BlackboxCallbacksBase;
class MessageQueueBase;

struct blackbox_context_t;


class BlackboxTask : public TaskBase {
public:
    BlackboxTask(uint32_t task_interval_microseconds, Blackbox& blackbox, MessageQueueBase& message_queue, blackbox_context_t& context);
public:
    static BlackboxTask* create_task(task_info_t& task_info, Blackbox& blackbox, MessageQueueBase& message_queue, blackbox_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static BlackboxTask* create_task(Blackbox& blackbox, MessageQueueBase& message_queue, blackbox_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
private:
    // class is not copyable or moveable
    BlackboxTask(const BlackboxTask&) = delete;
    BlackboxTask& operator=(const BlackboxTask&) = delete;
    BlackboxTask(BlackboxTask&&) = delete;
    BlackboxTask& operator=(BlackboxTask&&) = delete;
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    uint32_t _task_interval_milliseconds;
    Blackbox& _blackbox;
    MessageQueueBase& _message_queue;
    blackbox_context_t& _context;
};
