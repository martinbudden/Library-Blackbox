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

Both Betaflight and Cleanflight are licensed under the GNU GPL

This library also contains Kustaa Nyholm's `printf` implementation, which is included under
the terms of its license and includes that implementation's copyright notice, conditions and disclaimer.

The original Betaflight copyright notice is included in License.txt and the program files,
as per the GNU GPL "keep intact all notices” requirement.
