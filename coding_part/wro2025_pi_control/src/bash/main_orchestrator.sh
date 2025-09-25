#!/bin/bash

# main_orchestrator.sh
# Main orchestrator script for the WRO 2025 robot.
# Manages the overall state machine (IDLE, WAITING_START, WAITING_GO, RUNNING, FINISHED, ERROR).
# *** CORRECTED COMMAND IDs for GO signal ***

set -e

# Source configuration and utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/utils.sh"
source "$SCRIPT_DIR/../../config/main_config.conf"

# --- State Variables ---
# These will be managed using the state file
# state=$(read_json_state "$STATE_FILE" "state") # Example usage


main() {
    log "INFO" "Starting WRO 2025 Main Orchestrator..."

    # Create log directory
    mkdir -p "$LOG_DIR"

    # Initialize state file if needed
    if [[ ! -f "$STATE_FILE" ]]; then
         echo '{"state": "IDLE", "lap_count": 0, "challenge_type": "OPEN", "parking_zone_saved": false, "parking_zone_data": null, "parking_aligned": false, "parking_distance": -1}' > "$STATE_FILE"
    fi

    # Create command queue FIFO if needed (hardware_interface.sh also does this, but good to ensure)
    if [[ ! -p "$COMMAND_QUEUE_FILE" ]]; then
        mkfifo "$COMMAND_QUEUE_FILE"
    fi

    # Main state machine loop
    while true; do
        local current_state
        current_state=$(read_json_state "$STATE_FILE" "state")
        log "INFO" "Current state: $current_state"

        case $current_state in
            "IDLE")
                # Wait for physical start button press (detected by Nano and reported via state file)
                local started
                started=$(read_json_state "$STATE_FILE" "started")
                if [[ "$started" == "true" ]]; then
                    log "INFO" "Physical start button pressed acknowledged by Nano."
                    update_json_state "$STATE_FILE" "state" "WAITING_GO" "string"
                    # Send the GO signal to the Nano via the command queue
                    echo "GO" > "$COMMAND_QUEUE_FILE" # *** CORRECTED COMMAND ***
                fi
                ;;
            "WAITING_GO")
                # Wait for GO signal from this script (after button press) to be received by Nano
                local go_received
                go_received=$(read_json_state "$STATE_FILE" "go_received")
                if [[ "$go_received" == "true" ]]; then
                    log "INFO" "GO signal acknowledged by Nano. Starting mission."
                    # Start other processes (vision, navigation)
                    # Example: python3 "$SCRIPT_DIR/../python/vision_processor.py" &
                    # Example: "$SCRIPT_DIR/navigation_controller.sh" &
                    update_json_state "$STATE_FILE" "state" "RUNNING" "string"
                    update_json_state "$STATE_FILE" "lap_count" 0 "number" # Reset lap count
                fi
                ;;
            "RUNNING")
                # Main control loop managed by navigation_controller.sh
                # This orchestrator monitors for finish conditions (time, laps, parking)
                local lap_count
                lap_count=$(read_json_state "$STATE_FILE" "lap_count")
                if (( lap_count >= LAP_COUNT_TARGET )); then
                    local challenge_type
                    challenge_type=$(read_json_state "$STATE_FILE" "challenge_type")
                    if [[ "$challenge_type" == "OBSTACLE" ]]; then
                        log "INFO" "Lap target reached in OBSTACLE challenge. Transitioning to PARKING."
                        update_json_state "$STATE_FILE" "state" "PARKING" "string"
                    else
                        log "INFO" "Lap target reached in OPEN challenge. Transitioning to FINISHED."
                        update_json_state "$STATE_FILE" "state" "FINISHED" "string"
                    fi
                fi
                # Check for errors reported by Nano or other scripts
                local error
                error=$(read_json_state "$STATE_FILE" "error")
                if [[ "$error" == "true" ]]; then
                    log "ERROR" "Error state reported. Transitioning to ERROR."
                    update_json_state "$STATE_FILE" "state" "ERROR" "string"
                fi
                ;;
            "PARKING")
                # Parking logic managed by navigation_controller.sh
                # Monitor for parking completion (state updated by nav controller)
                local parking_completed
                # Assuming navigation controller updates state to FINISHED upon successful parking
                # Or, check a specific parking status flag if needed
                # For now, assume state changes to FINISHED after parking attempt
                # Check state again in the next loop iteration
                ;;
            "FINISHED")
                # Stop all processes, send stop command to Nano
                log "INFO" "Mission finished. Sending STOP command."
                echo "STOP" > "$COMMAND_QUEUE_FILE"
                # Stop vision, navigation processes if running (implement process management if needed)
                log "INFO" "Mission finished successfully."
                # Exit or wait for restart signal (for now, exit)
                break # For testing, exit after finish
                ;;
            "ERROR")
                # Emergency stop, log error, wait for reset
                log "ERROR" "Error state reached. Sending STOP command."
                echo "STOP" > "$COMMAND_QUEUE_FILE" 2>/dev/null || true
                # Potentially trigger hardware reset or wait for manual intervention
                # For now, break
                log "ERROR" "Orchestrator exiting due to error."
                break # For testing, exit on error
                ;;
            *)
                log "ERROR" "Unknown state: $current_state"
                break
                ;;
        esac

        sleep 0.1 # Main loop frequency
    done

    log "INFO" "Main Orchestrator exiting."
}

# --- Cleanup ---
cleanup() {
    log "INFO" "Shutting down orchestrator..."
    # Kill child processes if any (implement process management if needed)
    # Send stop command to Nano
    echo "STOP" > "$COMMAND_QUEUE_FILE" 2>/dev/null || true
    log "INFO" "Orchestrator shutdown complete."
}

trap cleanup EXIT
trap 'exit 0' SIGTERM SIGINT

main "$@"