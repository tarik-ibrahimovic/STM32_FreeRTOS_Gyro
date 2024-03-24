# STM32_FreeRTOS_Gyro

This repository represents a Real Time Systems course final project. The primary emphasis is on the Real Time Systems aspect of execution, with hardware and tools taking a supporting role in the background. 

## Prerequisites

The project is designed for the ST Microelectronics **STM32 STM32F072RB** board, utilizing **STM32CubeIDE** as the development environment. Given the project's focus on Real-Time Systems, it's interesting to observe how jobs are scheduled. Monitoring is done using **SEGGER SystemView**, while previously reflashing the board to a **J-Link** configuration.

## System operation - tasks

Below is described a set of tasks, be it periodic or aperiodic, imposed on the Real Time System, where priority can be revised if neccessary. Obvious limitations arise when certain tasks are non-preemptable, especially when multiple tasks are simultaneously accessing shared resources such as memory, UART, and Gyro Sensor. Handling this constraint is a shared semaphore mutex, marked in tasks that make use of it:
- Priority 1
    - Storing last 30 Gyro sensor value samples (periodic) 
    - Change LED values according to direction of rotation (periodic) (mutex)
    - Send Gyro Sensor value via UART (periodic) (mutex)
- Priority 2
    - Touch slider sensor events (aperiodic) 
- Priority 3 
    - Sampling Gyro Sensor value (periodic) (mutex)
- Priority 4
    - On a 3-second user button press send the stores samples via UART (aperiodic) 
    - Send Gyro measurement when there is a >5% change in value (aperiodic) (mutex)

Illustrating the implementation is the inserted block diagram, with inputs on top, processing in the middle, and outputs on the bottom.
![System implementation block diagram](/0.doc/IO.png)

## Example execution
When executed, here is an example of the expected output on the serial port, and this is how SystemView should display task execution.
![Serial port read](/0.doc/serial.png)
![SystemView task execution](/0.doc/IO.png)



