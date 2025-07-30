# Blackbox Library [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

This library is a port of the Betaflight Blackbox library to C++. (Betaflight is written in C).
It also contains Kustaa Nyholm's `printf` implementation.

The main changes in this library are:

1. Code has been converted to C++.
2. Dependencies on Betaflight internals (ie parameter groups, configs, features, sensors etc)
   have been removed and abstracted into callbacks, so this library can be used on its own.

## Notes on Blackbox tasks and scheduling

Betaflight uses non-preemptive cooperative scheduling. This means tasks undertake activities that
take a long time need to yield while undertaking those activities. Betaflight uses two common design patterns
to do this:

1. Splitting an activity into reasonable chunks and using a state machine to sequence through those chunks.
2. Making asynchronous calls to a function with a callback for when that function completes.

In Betaflight, Blackbox runs as a subtask of the MainPidLoop task, which typically runs at 8kHz. Blackbox
can be configured to run at a submultiple of that frequency (ie 4kHz, 2kHz, etc).

## Notes on the port

This is very much a "minimal effort" port - just enough work was done to get it working for ProtoFlight and SelfBalancingRobot.

This means the code is a curious mix of both my style and Betaflight style. It is also a curious mix of C-like code and C++-like code.

## License Information

This library is a port (modification) of the Blackbox implementation
in Betaflight (which itself was a port of the Cleanflight implementation).

I believe the original implementation was done by [Nicholas Sherlock (aka thenickdude)](https://www.nicksherlock.com/),
[source code](https://github.com/thenickdude/blackbox). This was licensed under the GNU GPL v3.

Both Betaflight and Cleanflight are licensed under the GNU GPL

This library also contains Kustaa Nyholm's `printf` implementation, which is included under
the terms of its license and includes that implementation's copyright notice, conditions and disclaimer.

The original Betaflight copyright notice is included in License.txt and the program files,
as per the GNU GPL "keep intact all notices” requirement.

## Simplified Class Diagram

All writing to the serial device is done via the `BlackboxEncoder`

```mermaid
classDiagram
    class BlackboxEncoder {
        write(uint8_t value)
    }
    class Blackbox {
        <<abstract>>
        writeSystemInformation() write_e *
        virtual update() uint32_t
    }
    class BlackboxCallbacksBase {
        <<abstract>>
        loadSlowState() *
        loadMainState() *
    }
    class BlackboxSerialDevice {
        <<abstract>>
    }
    class BlackboxFIFO_Base {
        <<abstract>>
        update(timeUs gyro, gyroUnfiltered, acc) uint32_t *
    }
    class BlackboxSerialDeviceSDCard["BlackboxSerialDeviceSDCard(eg)"]

    Blackbox *-- BlackboxEncoder : calls write
    BlackboxEncoder o-- BlackboxSerialDevice : calls write
    Blackbox o-- BlackboxSerialDevice : calls open close
    %%BlackboxSerialDevice --o BlackboxEncoder : calls write
    BlackboxSerialDevice <|-- BlackboxSerialDeviceSDCard
    Blackbox o-- BlackboxCallbacksBase
    BlackboxCallbacksBase o-- BlackboxFIFO_Base

    TaskBase <|-- BlackboxTask
    class BlackboxTask {
        +loop()
        -task() [[noreturn]]
    }
    BlackboxTask o-- Blackbox : calls update
```
