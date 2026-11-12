# stm32-pid

Lightweight PID controller implementation for embedded systems using fixed sample time execution and integral anti-windup.

---

## Project Overview

This project provides a generic PID controller implementation written in C for embedded applications.

The controller features:

- Proportional, Integral and Derivative control
- Integral clamping anti-windup
- Configurable output limits
- Fixed sample time operation

The implementation is suitable for STM32 projects but can be used on any microcontroller platform.

---

## Key Features

- Simple API
- Derivative kick prevention
- Integral anti-windup
- Configurable limits
- Fixed sample-time implementation
- No hardware dependencies

---

## Repository Structure

```text
stm32-pid/
├── Core/
│   ├── Inc/
│   │   └── pid.h
│   └── Src/
│       └── pid.c
├── LICENSE
└── README.md
```

---

## Integration

Copy the files into your project:

```text
Core/Inc/pid.h
Core/Src/pid.c
```

Include:

```c
#include "pid.h"
```

---

## Configuration

```c
PID_Config_t pid_cfg =
{
    .Kp = 1.0f,
    .Ki = 0.5f,
    .Kd = 0.05f,

    .Ts = 0.001f,

    .Setpoint = 100.0f,

    .output_min = -100.0f,
    .output_max = 100.0f
};
```

---

## Initialization

```c
PID_Controller_t pid;

PID_Init(&pid, &pid_cfg);
```

---

## Controller Update

```c
float measurement = GetSpeed();

float output =
    PID_Update(&pid,
               measurement);
```

---

## Change Setpoint

```c
PID_SetSetpoint(&pid,
                250.0f);
```

---

## Reset Controller

```c
PID_Reset(&pid,
          measurement);
```

---

## Output Limits

```c
PID_SetOutputLimits(&pid,
                    -50.0f,
                    50.0f);
```

---

## Anti-Windup

The controller implements integral clamping anti-windup.

When the controller output is saturated, integration is only allowed if the accumulated error drives the output back toward the valid operating range.

---

## Typical Applications

- Motor speed control
- Position control
- Temperature control
- Power regulation
- Process control
- Motion control

---

## Notes

- Uses derivative-on-measurement
- Prevents derivative kick
- Assumes a fixed sample period (Ts)

The controller assumes a fixed sampling period. The update function must be called periodically at the configured sample time Ts. Irregular update intervals will affect controller performance.

---

## Future Improvements

- Gain scheduling
- Feed-forward support
- Unit tests

---
