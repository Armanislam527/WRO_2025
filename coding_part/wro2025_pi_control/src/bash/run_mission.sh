#!/bin/bash

# run_mission.sh
# Top-level script to start the WRO 2025 mission.
# Launches Hardware Interface, Vision Processor, and Main Orchestrator.

set -e # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/utils.sh"
source "$SCRIPT_DIR/../../config/main_config.conf"

# --- Process Management ---
declare -A pids
declare -a process_names=("HardwareInterface" "VisionProcessor" "NavigationController" "MainOrchestrator")

# --- Function to start a process ---
start_process() {
    local script_path=$1
    local name=$2
    local log_file="$LOG_DIR/${name,,}.log" # Convert name to lowercase for log file

    log "INFO" "Starting $name..."
    # Run the script in the background, redirect output to log file
    "$script_path" > "$log_file" 2>&1 &
    local pid=$!
    pids["$name"]=$pid
    log "INFO" "$name started with PID $pid."
}

# --- Function to stop all processes ---
stop_processes() {
    log "INFO" "Stopping all processes..."
    for name in "${process_names[@]}"; do
        local pid=${pids[$name]}
        if [[ -n "$pid" ]]; then
            log "INFO" "Sending SIGTERM to $name (PID $pid)..."
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    # Wait a bit for graceful shutdown
    sleep 2

    # Force kill if still running
    for name in "${process_names[@]}"; do
        local pid=${pids[$name]}
        if [[ -n "$pid" ]]; then
            log "INFO" "Sending SIGKILL to $name (PID $pid) if still running..."
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done
    log "INFO" "All processes stopped."
}

# --- Signal Traps for Graceful Shutdown ---
trap stop_processes EXIT
trap 'exit 0' SIGTERM SIGINT

# --- Main Execution ---
log "INFO" "Starting WRO 2025 Mission Control Suite..."

# Create log directory
mkdir -p "$LOG_DIR"

# Initialize shared files if needed (or let components do it)
# State file is initialized by main_orchestrator.sh
# Command queue is created by hardware_interface.sh or orchestrator
# Sensor data file is initialized by hardware_interface.sh
# Vision output file will be created by vision_processor.py

# Start processes in the correct order (or concurrently, as they are mostly independent)
# Hardware Interface needs to be ready for commands from Navigation
# Vision Processor needs to be ready for Navigation to read its output
# Navigation Controller needs to be ready for Orchestrator to potentially trigger parking
# Main Orchestrator manages the overall state and starts the sequence

start_process "$SCRIPT_DIR/hardware_interface.sh" "HardwareInterface"
sleep 1 # Brief delay to ensure hardware interface is listening
# Ensure vision_processor is executable and has shebang
chmod +x "$SCRIPT_DIR/../python/vision_processor.py" 2>/dev/null || true
start_process "$SCRIPT_DIR/../python/vision_processor.py" "VisionProcessor"
sleep 1 # Brief delay to ensure vision processor is running
start_process "$SCRIPT_DIR/navigation_controller.sh" "NavigationController"
sleep 1 # Brief delay to ensure navigation controller is running
start_process "$SCRIPT_DIR/main_orchestrator.sh" "MainOrchestrator"

# Wait for the orchestrator (which manages the mission state) to finish
# The other processes should be stopped gracefully by the orchestrator or the trap
orchestrator_pid=${pids["MainOrchestrator"]} # *** FIXED: Removed 'local' ***
if [[ -n "$orchestrator_pid" ]]; then
    log "INFO" "Waiting for Main Orchestrator (PID $orchestrator_pid) to finish..."
    wait "$orchestrator_pid"
    log "INFO" "Main Orchestrator finished."
else
    log "ERROR" "Main Orchestrator PID not found!"
fi

# The trap 'stop_processes' will be called automatically when this script exits
log "INFO" "Mission Control Suite run script finished."