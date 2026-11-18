# Gyropode – self-balancing robot on STM32

Prototype of a two-wheeled self-balancing robot (inverted pendulum), built on an
STM32 Nucleo-F401RE. The firmware is written in C with STM32CubeIDE and the HAL,
with no RTOS.

Embedded systems project – Master Automatique, Robotique (MARS), M1,
Polytech Orléans, 2025/26.
Team: El Mahdi Hariri, Adam Boukri, Lucas Gaudin – supervised by Dominique Nelson-Gruel.

<!-- Add a line here about your own part of the work, e.g. "My part: ..." -->

<p align="center">
  <img src="docs/prototype.jpg" width="420" alt="Prototype: Nucleo-F401RE, sensors, L298N and motors">
</p>

## How it works

The robot is an inverted pendulum: on its own it falls. To stay upright the
wheels have to move under the centre of mass, in the direction it is falling.

```
 MMA7660 (accel) ─┐                      ┌──────────────┐     ┌─────┐     ┌───────┐
                  ├─ I2C ─► pitch angle ─► complementary├─► θ ─►│ PID ├─► u ─► L298N ├─► 2 DC motors
 ITG-3200 (gyro) ─┘                      │    filter    │     └─────┘     └───────┘
                                         └──────────────┘        (TIM3 PWM, 1 kHz)
```

Everything runs in a 100 Hz loop (every 10 ms):

1. **Read the IMU** over I2C.
2. **Estimate the tilt angle θ** with a complementary filter.
3. **Compute the motor command** with a PID on θ (setpoint = 0°).
4. **Drive the motors**: PWM duty cycle on the L298N enable pins, direction on IN1/IN2.

<p align="center">
  <img src="docs/architecture.png" width="650" alt="System architecture: sensors, STM32, L298N, motors and power supplies">
</p>

### Angle estimation

Neither sensor is good enough on its own:

- the **accelerometer** gives an absolute angle from the direction of gravity,
  but it is noisy and disturbed every time the robot accelerates;
- the **gyroscope** is smooth, but integrating its rate drifts over time.

The complementary filter combines both:

```
θ = α · (θ + ω_gyro · dt) + (1 − α) · θ_accel        with α = 0.98
```

In practice it is a high-pass filter on the gyro and a low-pass filter on the
accelerometer. With dt = 10 ms the time constant is about 0.5 s.

Sensor conversions:

```
a [g]   = (raw − offset) / 21          (MMA7660: 21 LSB = 1 g)
ω [°/s] = (raw − offset) / 14.375      (ITG-3200, ±2000 °/s range)
```

### Control

- PID with gains tuned by hand on the robot: Kp = 25, Ki = 0.5, Kd = 2.
- The derivative is taken on the measurement, not on the error.
- Anti-windup: the integral term is clamped.
- **Friction compensation**: the motors don't move below ~30 % duty cycle, so
  any non-zero command smaller than that is raised to 30 %.
- **Fall detection**: beyond 45° the motors are cut. The loop restarts on its
  own when the robot is put back upright (|θ| < 5°).

## Hardware

| Part | Role |
|---|---|
| STM32 Nucleo-F401RE (Cortex-M4, run at 84 MHz) | microcontroller |
| Grove MMA7660FC | 3-axis accelerometer, ±1.5 g, 6 bits, I2C address 0x4C |
| Grove ITG-3200 | 3-axis gyroscope, ±2000 °/s, I2C address 0x68 |
| L298N | dual H-bridge, up to 2 A per motor, 1 kHz PWM |
| 2 × DC gear motors (5–6 V) + wheels | actuators |
| 2 × 4.7 kΩ resistors | I2C pull-ups on SCL and SDA |

Power: the Nucleo is powered over USB (5 V) and the motors from a separate 6 V
supply, with a common ground.

### Wiring

| Signal | STM32 pin |
|---|---|
| I2C1 SCL / SDA (both sensors) | PB6 / PB7 |
| Left motor PWM (L298N ENA) | TIM3_CH1 – PA6 |
| Right motor PWM (L298N ENB) | TIM3_CH2 – PA7 |
| Left motor IN1 / IN2 | PA0 / PA1 |
| Right motor IN3 / IN4 | PA4 / PB0 |
| Debug UART (USART2 TX / RX, ST-LINK virtual COM port) | PA2 / PA3 |
| Status LED | PA5 |

## Development steps

The system was brought up one block at a time, each one checked before
moving to the next:

1. **Motors and PWM** – two motors had broken wires and were re-soldered. The
   TIM3 PWM signal was checked on an oscilloscope, then the L298N direction
   logic (IN1/IN2) was tested with a forward / reverse / stop sequence.
2. **Accelerometer** – I2C configured in CubeMX, link checked, raw values
   converted to g and calibrated (offsets measured at rest, 1 g kept on Z).
3. **Gyroscope** – same I2C bus, identity check (WHO_AM_I), offset measured at
   rest and removed, conversion to °/s.
4. **Fusion** – both sensors on the same bus (4.7 kΩ pull-ups), complementary
   filter, angle checked live.
5. **PID** – balance loop and hand tuning of the gains.

Debug tools: STM32CubeMonitor (live plots of the variables), Tera Term (UART
output), and an oscilloscope for the PWM.

| PWM on the oscilloscope | Accelerometer in CubeMonitor | Fused signals in CubeMonitor |
|---|---|---|
| <img src="docs/pwm_oscilloscope.jpg" width="260"> | <img src="docs/accelerometer_cubemonitor.png" width="260"> | <img src="docs/fusion_cubemonitor.png" width="260"> |

## Code structure

```
Core/
├── Src/
│   ├── main.c    init, 100 Hz control loop, fall detection, telemetry
│   ├── imu.c     MMA7660 and ITG-3200 drivers, calibration, complementary filter
│   ├── pid.c     generic PID (anti-windup, derivative on measurement)
│   └── motor.c   one L298N channel: direction + PWM duty cycle
└── Inc/
    ├── imu.h  pid.h  motor.h  main.h
```

`pid.c` and `motor.c` don't depend on this robot and can be reused as they are.
The other files in `Core/` are generated by STM32CubeMX.

## Build and run

1. Open the project in **STM32CubeIDE** (File → Open Projects from File System).
2. Enable float printing for the telemetry:
   Project → Properties → C/C++ Build → Settings → MCU Settings →
   tick *Use float with printf from newlib-nano*.
3. Build and flash.
4. Open a serial terminal at **115200 baud** on the ST-LINK COM port.
5. **Hold the robot upright and still** when it starts: it calibrates for a few
   seconds, and that position becomes its 0°.
6. Let go: the loop starts as soon as the robot is close to vertical.

## Telemetry

Every 50 ms the board sends a CSV line on the UART:

```
time_ms,pitch,accel_pitch,pitch_rate,pid_out,duty
```

It can be logged to a file and plotted to tune the gains or to compare the raw
accelerometer angle with the filtered one. The same variables can also be
watched live in STM32CubeMonitor.

## Limitations and next steps

- **No wheel encoders**: only the angle is controlled, so the robot slowly
  drifts forward or backward. Encoders and a second loop on wheel speed
  (cascade control) would keep it in place.
- **Sensor fusion**: a Kalman filter would be more robust than the
  complementary filter. The MMA7660 only has 6 bits (~3° steps), so a modern
  IMU (MPU-6050, LSM6DS3) would also give a cleaner angle.
- **Hand-tuned gains**: a model of the pendulum (or an LQR design) would give a
  better starting point than trial and error.
- **Timing**: the loop is scheduled with `HAL_GetTick()` (1 ms resolution) and
  the UART transmit is blocking. A timer interrupt for the loop and DMA for the
  UART would make the period more regular.
- **Mechanics and autonomy**: a 3D-printed chassis (first plates already
  modelled), an on-board battery, Bluetooth for remote control and
  wireless debugging, then obstacle avoidance.
