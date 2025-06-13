# Valve Control Board

This is the software responsible for maintaining control of the solenoid valves XV-4 and XV-6 during flight.

It does the following:

1. Wait for the continuity line to read low (disconnected)
2. Turn on the MOSFET switches that output 24V battery power on the solenoid power lines
3. Begin a timer for the flight duration (configurable, somewhere in the minutes range)
4. Once the timer ends, turn off the MOSFET switches

This code is written in bare-metal C to be as low-resource as possible, while still making use of the AVR build
tool-chain conveniences. Should also be more legible than assembly.

## Visualization

This is a visualization of the valve control logic using Finite State Machine (FSM) notation:

![Valve control FSM](./docs/ValveControlFSM.png)
