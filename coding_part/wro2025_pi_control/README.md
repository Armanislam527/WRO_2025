# WRO 2025 Self-Driving Car - Raspberry Pi Control

This repository contains the Bash-based control software for the Raspberry Pi 0 2W,
designed to interface with an Arduino Nano for the WRO 2025 Future Engineers Self-Driving Cars competition.

## Structure

-   `src/bash/`: Main Bash scripts (orchestrator, hardware interface, etc.).
-   `src/python/`: Python helpers (vision, sensor fusion).
-   `data/shared_state.json`: Current mission state.
-   `data/sensor_data.json`: Latest sensor readings from the Nano.
-   `data/nav_commands.json`: Commands from navigation to hardware interface.
-   `config/main_config.conf`: Main configuration file.

## Setup

1. Ensure dependencies are installed (jq, python3, opencv-python, numpy, pyserial).
2. Create necessary directories and files.
3. Upload the corresponding Arduino Nano firmware.

## Running

Use the main orchestrator script.
