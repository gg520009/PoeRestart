# PoeRestart - PoE Power Controller

[中文版](readme-zh.md)

This project implements a Power over Ethernet (PoE) power management controller using an STM32 microcontroller. It is designed to interface with the **LTC4294** PD (Powered Device) interface controller to manage high-power negotiations (up to 71W) and control downstream power distribution safely.

## Features

- **Power Negotiation Monitoring**: continuously monitors the T2P and PWGD signals from the LTC4294 to determine the available power budget.
- **Soft-Start & Sequence Control**: Manages the enable signals for the DC-DC converter and output relay to prevent inrush current and ensure stable startup.
- **Status Indication**: LED blink patterns indicate the system's operating state (Waiting, Negotiating, Power Good, or Fault).
- **Protection**: Includes an independent watchdog (IWDG) and fault detection logic to disconnect power in case of negotiation failure or instability.
- **DMA-Optimized**: Uses DMA for ADC sampling and UART logging to minimize CPU usage.

## Hardware Configuration

The system is based on an STM32 microcontroller (e.g., STM32F0/G0 series) running at 64MHz (HSI/PLL).

### Pinout Mapping

| Pin | Function      | Type  | Description                                      |
| --- | ------------- | ----- | ------------------------------------------------ |
| PA0 | **T2P**       | ADC   | Sample LTC4294 T2P signal (Power availability)   |
| PA1 | **PWGD**      | ADC   | Sample LTC4294 Power Good signal (>1.5V = Good)  |
| PA2 | **Powerkeyin**| In    | Input Control Signal (Active Low/High Config)    |
| PA4 | **LB16F1**    | Out   | LED Control Indicator (Follows Powerkeyinstate)  |
| PA5 | **AP**        | Out   | AP Sequence Output Control                       |
| PA6 | **DC-DC EN**  | Out   | DC-DC Enable Control (**Low** = Enable)          |
| PA7 | **RELAY**     | Out   | Output Relay Control (**High** = Close/On)       |
| PB0 | **LED**       | Out   | Status LED (Open-Drain, Low = On)                |
| PB3 | **USART1_TX** | UART  | Debug Log Output (115200 bps)                    |

## Software Logic

The system runs a state machine on a 10µs time base.

### State 1: Wait for Power Good

- **Indicator**: Fast Blink (1 Hz / 0.5s ON, 0.5s OFF)
- **Behavior**: Monitors `PA1` (PWGD).
  - If PWGD > 1.5V, it tentatively enables the DC-DC and Relay.
  - If PWGD remains stable for ~3 seconds, the system transitions to **State 2**.
  - If PWGD is unstable, it keeps the output disabled.

### State 2: Power Negotiation Check

- **Indicator**: Medium Blink (0.5 Hz / 1s ON, 1s OFF)
- **Behavior**: Analyzes the T2P signal on `PA0` to determine the allocated power.
  - Collects 8192 samples (approx. 82ms window).
  - Checks if the T2P average voltage corresponds to the **71W** Class (Approx. 2.3V - 2.6V).
  - **Success**: If 71W is valid, transitions to **State 3**.
  - **Failure**: If power negotiation fails (e.g., < 71W), transitions to **State 4**.

### State 3: Normal Operation (High Power)

- **Indicator**: Slow Blink (~0.16 Hz / 3s ON, 3s OFF)
- **Behavior**:
  - **Power Output**: **Enabled** (PA6 Low, PA7 High).
  - The system remains in this state as long as power is stable.

### State 4: Fault / Low Power Mode

- **Indicator**: LED Off
- **Behavior**:
  - **Power Output**: **Disabled** (PA6 High, PA7 Low).
  - The system stays in this state for 3 seconds before resetting to **State 1**.

### Additional Control Logic (Parallel)

The system runs additional control tasks in parallel with the main state machine:

#### 1. Powerkeyin Control (PA2)

- **Input Monitoring**: Continuously monitors `PA2` with a **500ms software debounce** filter.
- **State Update**: Updates global `Powerkeyinstate` (1 = Active/High, 0 = Inactive/Low).

#### 2. LB16F1 Indicator (PA4)

- Directly reflects the `Powerkeyinstate`.
- **ON**: `Powerkeyinstate` == 1.
- **OFF**: `Powerkeyinstate` == 0.

#### 3. AP Sequence Control (PA5)

Non-blocking sequence trigger on `Powerkeyinstate` transitions:

- **0 -> 1 Transition**: `OFF (10ms)` -> `ON (300ms)` -> `OFF (10ms)` -> `IDLE (OFF)`.
- **1 -> 0 Transition**: `OFF (10ms)` -> `ON (8000ms)` -> `OFF (10ms)` -> `IDLE (OFF)`.

## Build Instructions

This project is generated with STM32CubeMX and can be built using **STM32CubeIDE**.

1. Open the project directory in STM32CubeIDE.
2. Ensure the correct MCU target is selected in the project settings.
3. Build the project (Ctrl+B).
4. Flash the binary to the target board.

## License

This project is licensed under the terms provided in the LICENSE file. If no license is present, it is provided AS-IS.
Copyright (c) 2026 STMicroelectronics.
