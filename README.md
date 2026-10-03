# StarPi Flight Simulation

A real-time flight telemetry replay, state estimation, and visualization tool for the **StarPi / StarFly** rocket avionics.

![StarFly Flight Simulation](<Schermata 3.png>)

## Features

- **Telemetry Replay & Streaming:** Replays flight data from CSV logs (`imu.csv`, `baro.csv`, `filteredDataInfo.csv`) at 100 Hz and can stream telemetry over serial (`/dev/ttyUSB*` / `/dev/ttyACM*`) or stdout.
- **State Estimation:**
  - **Orientation:** Mahony AHRS filter (`Adafruit_AHRS`).
  - **Altitude & Vertical Velocity:** 1D Kalman Filter combining barometric altitude and longitudinal acceleration.
- **Flight State Machine (FSM):** Tracks flight phases (`IDLE`, `BOOST`, `COAST`, `DROGUE`, `MAIN`, `TOUCHDOWN`) and parachute deployment triggers.
- **Ground Diagnostics:** Pre-flight variance and validity checks for IMU and barometer sensors.
- **Live GUI:** Interactive real-time plots built with [Dear ImGui](https://github.com/ocornut/imgui) and [ImPlot](https://github.com/epezent/implot) (GLFW + OpenGL 3).

## Prerequisites

- C++17 compiler (`g++`)
- GLFW3 & OpenGL development libraries:
  ```bash
  sudo apt-get install libglfw3-dev libgl1-mesa-dev
  ```

## Build & Run

1. **Compile:**
   ```bash
   make
   ```

2. **Run:**
   ```bash
   # Run with a specific flight data directory
   ./plotter -d main_export_09182026_022604

   # Show options
   ./plotter -h
   ```
