#!/bin/bash

# navigation_controller.sh (Optimized)
# Core logic for autonomous driving: line following, lap counting, obstacle/sign handling, parking.
# Reads sensor data from JSON file, potentially vision output, calculates commands, writes to command queue.
# Optimized for robustness and WRO 2025 rules compliance.

set -e # Exit on any error

# --- Source Configuration and Utilities ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/utils.sh"
# Correct path to config (two levels up to project config dir)
if [[ -r "$SCRIPT_DIR/../../config/main_config.conf" ]]; then
    source "$SCRIPT_DIR/../../config/main_config.conf"
else
    # Fallback to sibling config dir if present
    [[ -r "$SCRIPT_DIR/../config/main_config.conf" ]] && source "$SCRIPT_DIR/../config/main_config.conf"
fi

# --- Constants (could be moved to config or tuned dynamically) ---
LAP_COUNT_TARGET=3
MIN_DISTANCE_THRESHOLD=15 # cm - minimum distance to consider an obstacle/sign
PARKING_ALIGNMENT_TOLERANCE=3 # degrees - how close to 90 degrees for parking
PARKING_DISTANCE_THRESHOLD=18 # cm - distance from parking blocks/wall to start parking
PARKING_DURATION=6000 # milliseconds for parking maneuver (if needed)
# PID Controller Gains (Example - need tuning per challenge/vehicle)
# Using associative arrays for different modes/challenges if needed later
declare -A PID_GAINS_SPEED
PID_GAINS_SPEED[OPEN]="1.0 0.0 0.0" # kp ki kd
PID_GAINS_SPEED[OBSTACLE]="0.8 0.01 0.0"
declare -A PID_GAINS_STEER
PID_GAINS_STEER[OPEN]="1.2 0.0 0.0"
PID_GAINS_STEER[OBSTACLE]="1.0 0.0 0.0"

# --- Global Variables (Internal State) ---
current_lap=0
last_lap_completion_time=0
# Track heading using IMU gyro (simplified integration)
current_heading_rad=0.0
last_gyro_time=0
# Track position using IMU (simplified integration - prone to drift, mainly for lap counting aid)
last_pos_x=0.0
last_pos_y=0.0
current_pos_x=0.0
current_pos_y=0.0
last_imu_time=0
# Lap counting state
last_lap_zone_id="" # Identifier for the zone where the last lap finished (e.g., section ID)
current_zone_id=""  # Identifier for the current zone
# Sign handling state
sign_detected_time=0
sign_handling_active=false
sign_pass_direction="" # "left" or "right" relative to the robot
sign_avoidance_angle=0 # Angle offset to apply while avoiding sign
# Parking state
is_parking=false
parking_start_time=0
parking_state="SEARCHING" # "SEARCHING", "ALIGNING", "APPROACHING", "MANEUVERING", "FINISHED"
# PID State Variables
last_error_speed=0
integral_speed=0
last_error_steer=0
integral_steer=0
# Timers
last_sensor_time=0
last_control_time=0
# Lap detection thresholds (based on heading change)
LAP_HEADING_CHANGE_THRESHOLD=6.0 # radians (approx 344 degrees, allowing for slight errors, ~5.5 turns for 3 laps if each lap is ~1.88 rad turn)

# --- PID Controller Helper Function ---
calculate_pid_output() {
    local current_value=$1
    local setpoint=$2
    local kp=$3
    local ki=$4
    local kd=$5
    local last_error_ref=$6
    local integral_ref=$7
    local dt=$8 # Time delta in seconds

    local error
    error=$(echo "$setpoint - $current_value" | bc -l)

    local integral
    integral=$(echo "$integral_ref + $error * $dt" | bc -l)

    local derivative
    derivative=$(echo "($error - $last_error_ref) / $dt" | bc -l)

    local output
    output=$(echo "$kp * $error + $ki * $integral + $kd * $derivative" | bc -l)

    # Return values via echo (caller needs to capture)
    echo "$output $integral $error"
}

# --- Update Internal Position/Heading based on IMU (Simplified) ---
update_imu_state() {
    local ax ay az gx gy gz
    ax=$(read_json_state "$SENSOR_DATA_FILE" "accelX")
    ay=$(read_json_state "$SENSOR_DATA_FILE" "accelY")
    az=$(read_json_state "$SENSOR_DATA_FILE" "accelZ")
    gx=$(read_json_state "$SENSOR_DATA_FILE" "gyroX")
    gy=$(read_json_state "$SENSOR_DATA_FILE" "gyroY")
    gz=$(read_json_state "$SENSOR_DATA_FILE" "gyroZ")

    local current_time
    current_time=$(date +%s.%3N)
    local dt_imu
    dt_imu=$(echo "$current_time - $last_imu_time" | bc -l)
    last_imu_time=$current_time

    if (( $(echo "$dt_imu > 0" | bc -l) )); then
        # Convert gyro Z (yaw rate) from deg/s * 1000 to rad/s
        local gyro_z_rad_s
        gyro_z_rad_s=$(echo "$gz / 1000.0 * 3.14159265358979323846 / 180.0" | bc -l)
        # Integrate heading (simplified, assumes Z-axis is up/down)
        current_heading_rad=$(echo "$current_heading_rad + $gyro_z_rad_s * $dt_imu" | bc -l)

        # Optional: Integrate velocity from accelerometer (requires more complex filtering/estimation)
        # For now, just track heading change for lap detection.
    fi
}

# --- Determine Current Zone ID (Simplified using heading change and distance) ---
# This is still a simplification. A more robust method might use visual markers or precise odometry.
determine_zone_id() {
    local front_dist right_dist back_dist left_dist
    front_dist=$(read_json_state "$SENSOR_DATA_FILE" "front")
    right_dist=$(read_json_state "$SENSOR_DATA_FILE" "right")
    back_dist=$(read_json_state "$SENSOR_DATA_FILE" "back")
    left_dist=$(read_json_state "$SENSOR_DATA_FILE" "left")

    # Use heading change and distance thresholds to estimate being in a corner vs straight
    # This is a placeholder - actual implementation needs track geometry knowledge
    # and more sophisticated sensor fusion.
    # For now, just return a dummy ID based on a simple rule.
    if (( front_dist > 50 && left_dist < 20 && right_dist < 20 )); then
        echo "STRAIGHT_A" # Example ID
    elif (( left_dist > 30 && right_dist > 30 )); then
        echo "CORNER_1"   # Example ID
    else
        echo "TRANSITION" # Example ID
    fi
}

# --- Check for Lap Completion based on IMU Heading Change ---
check_lap_completion() {
    local total_heading_change
    total_heading_change=$(echo "scale=10; $current_heading_rad - 0.0" | bc -l) # Assuming starting heading was 0

    # Use absolute value to handle potential -2pi to +2pi wrap-around, though integration drift makes this tricky
    # A better method might track heading relative to the start of the current lap or use visual markers.
    # For now, use a large threshold based on expected total turn angle for 3 laps.
    if (( $(echo "$total_heading_change > $LAP_HEADING_CHANGE_THRESHOLD" | bc -l) )); then
        # Potential lap detected based on heading
        # Also check if we are in a consistent "end-of-lap" zone (requires more complex zone logic)
        # For simplicity here, assume heading change indicates completion if we are past the start area.
        # This is highly dependent on track layout and IMU accuracy.
        # A more robust check might involve zone transitions AND heading change.
        local current_zone_now
        current_zone_now=$(determine_zone_id)
        if [[ "$current_zone_now" != "$last_lap_zone_id" ]]; then # Avoid immediate double-counting
            current_lap=$((current_lap + 1))
            log "INFO" "Potential Lap completed (IMU-based) ! Lap Count: $current_lap / $LAP_COUNT_TARGET. Total Heading Change: $total_heading_change rad."
            update_json_state "$STATE_FILE" "lap_count" "$current_lap" "number"
            last_lap_zone_id="$current_zone_now" # Update last lap zone

            if (( current_lap >= LAP_COUNT_TARGET )); then
                 log "INFO" "Lap target ($LAP_COUNT_TARGET) reached based on IMU. Preparing for end phase."
                 # Let the orchestrator handle state change based on lap count in main_orchestrator.sh
                 # Or trigger parking directly if in OBSTACLE mode here.
                 local challenge_type
                 challenge_type=$(read_json_state "$STATE_FILE" "challenge_type")
                 if [[ "$challenge_type" == "OBSTACLE" ]]; then
                     update_json_state "$STATE_FILE" "state" "PARKING" "string"
                 else
                     update_json_state "$STATE_FILE" "state" "FINISHED" "string"
                 fi
                 return 0 # Indicate lap target reached
            fi
        fi
    fi
    return 1 # Indicate lap target not reached
}


# --- Main Navigation Loop ---
main_loop() {
    log "INFO" "Starting Optimized Navigation Controller Loop..."

    # Wait for the state to be RUNNING before starting main logic
    while [[ "$(read_json_state "$STATE_FILE" "state")" != "RUNNING" ]]; do
        sleep 0.1
    done
    log "INFO" "Navigation Controller: State is RUNNING, starting main logic."

    local current_time
    local dt_control
    local desired_speed=0
    local desired_angle=90 # Center angle
    local challenge_type
    challenge_type=$(read_json_state "$STATE_FILE" "challenge_type")

    # Initialize IMU state tracking time
    last_imu_time=$(date +%s.%3N)

    while true; do
        current_time=$(date +%s.%3N)
        dt_control=$(echo "$current_time - $last_control_time" | bc -l)
        last_control_time=$current_time

        # --- 0. Check for Parking State Triggered by Orchestrator ---
        if [[ "$(read_json_state "$STATE_FILE" "state")" == "PARKING" ]]; then
            is_parking=true
            log "INFO" "Parking state triggered by orchestrator. Exiting main navigation loop to parking routine."
            break
        fi

        # --- 1. Read Sensor Data and Update IMU State ---
        if ! check_file "$SENSOR_DATA_FILE"; then
            log "WARN" "Sensor data file $SENSOR_DATA_FILE not readable, skipping iteration."
            continue
        fi
        update_imu_state # Update internal heading/position based on IMU

        # --- 2. Lap Counting Logic (IMU-based) ---
        if check_lap_completion; then
            continue # Skip other logic if lap target reached
        fi

        # --- 3. Read Vision Output (if available) ---
        local vision_output_file="/tmp/vision_output.json"
        local line_error=0
        local sign_detected=false
        local sign_type=""
        local sign_direction=""
        local parking_aligned=false
        local parking_distance=-1

        if [[ -r "$vision_output_file" ]]; then
            line_error=$(read_json_state "$vision_output_file" "line_error" 2>/dev/null || echo 0)
            sign_detected=$(read_json_state "$vision_output_file" "sign_detected" 2>/dev/null || echo "false")
            sign_type=$(read_json_state "$vision_output_file" "sign_type" 2>/dev/null || echo "null")
            sign_direction=$(read_json_state "$vision_output_file" "sign_direction" 2>/dev/null || echo "null")
            # Parking data might be read here if vision provides it directly
            # parking_aligned=$(read_json_state "$vision_output_file" "parking_aligned" 2>/dev/null || echo "false")
            # parking_distance=$(read_json_state "$vision_output_file" "parking_distance" 2>/dev/null || echo "-1")
        fi

        # --- 4. Sign Handling Logic (Obstacle Challenge) ---
        if [[ "$challenge_type" == "OBSTACLE" ]]; then
            if [[ "$sign_detected" == "true" && "$sign_type" != "null" && "$sign_direction" != "null" ]]; then
                if [[ "$sign_handling_active" == false ]]; then
                    log "INFO" "Traffic sign detected: $sign_type, direction: $sign_direction. Initiating avoidance."
                    sign_handling_active=true
                    sign_pass_direction="$sign_direction"
                    sign_detected_time=$current_time
                    # Determine avoidance angle offset (e.g., steer away from the side we need to pass)
                    if [[ "$sign_direction" == "left" ]]; then # Pass on the left, so steer right initially
                        sign_avoidance_angle=20
                    elif [[ "$sign_direction" == "right" ]]; then # Pass on the right, so steer left initially
                        sign_avoidance_angle=-20
                    fi
                fi
            else
                # If no sign detected and we were handling one, reset
                if [[ "$sign_handling_active" == true ]]; then
                    log "INFO" "Sign handling completed or lost. Resuming normal navigation."
                    sign_handling_active=false
                    sign_pass_direction=""
                    sign_avoidance_angle=0
                fi
            fi
        fi

        # --- 5. Navigation Logic (Open vs Obstacle) ---
        local sensor_error=0
        local front_dist
        front_dist=$(read_json_state "$SENSOR_DATA_FILE" "front")

        if [[ "$sign_handling_active" == true ]]; then
            # --- Sign Avoidance Mode ---
            desired_speed=$((MAX_MOTOR_SPEED / 3)) # Slow down during sign handling
            # Apply pre-calculated avoidance angle offset
            desired_angle=$((90 + sign_avoidance_angle))
            # Add logic to ensure the sign is passed correctly before exiting this mode
            # This might involve checking sensor data again or waiting a specific time/distance
            # after the initial detection time.
            # For now, assume it's handled by the reset logic above.
        else
            # --- Normal Navigation Mode (Line Following) ---
            # Use US sensors and vision line error for steering
            local left_dist right_dist
            left_dist=$(read_json_state "$SENSOR_DATA_FILE" "left")
            right_dist=$(read_json_state "$SENSOR_DATA_FILE" "right")
            sensor_error=$(( (left_dist - right_dist) / 2 )) # Simplified sensor error

            # Combine sensor error and vision error
            local combined_error
            combined_error=$(echo "$sensor_error + $line_error" | bc -l)

            # Get PID gains based on challenge type
            local kp_steer ki_steer kd_steer
            read kp_steer ki_steer kd_steer <<< "${PID_GAINS_STEER[$challenge_type]}"

            # Calculate steering PID output
            local steer_output
            local new_integral_steer new_error_steer
            read steer_output new_integral_steer new_error_steer <<< $(calculate_pid_output "$combined_error" 0 "$kp_steer" "$ki_steer" "$kd_steer" "$last_error_steer" "$integral_steer" "$dt_control")
            last_error_steer=$new_error_steer
            integral_steer=$new_integral_steer

            desired_angle=$((90 + steer_output))
            desired_angle=$((desired_angle > 150 ? 150 : (desired_angle < 30 ? 30 : desired_angle))) # Constrain servo

            # Speed control based on front distance
            if (( front_dist < MIN_DISTANCE_THRESHOLD )); then
                desired_speed=$((MAX_MOTOR_SPEED / 4)) # Slow down significantly near obstacles
            else
                desired_speed=$((MAX_MOTOR_SPEED * 3 / 4)) # Maintain a good speed, slightly less than max for responsiveness
            fi
        fi

        # Get PID gains for speed based on challenge type
        local kp_speed ki_speed kd_speed
        read kp_speed ki_speed kd_speed <<< "${PID_GAINS_SPEED[$challenge_type]}"
        # Speed PID could be implemented here if desired, using desired_speed as setpoint
        # and current speed (from Nano or estimated) as process variable.
        # For now, we use desired_speed directly, with obstacle-based reduction above.

        # --- 6. Send Commands to Hardware Interface ---
        # log "DEBUG" "Sending Command - Speed: $desired_speed, Angle: $desired_angle" # Debug log
        echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
        echo "SERVO $desired_angle" > "$COMMAND_QUEUE_FILE"

        # --- 7. Brief Pause for Loop Frequency ---
        # Adjust sleep time based on desired control loop frequency (e.g., 20-50 Hz is often sufficient for this type of vehicle)
        sleep 0.04 # 40ms sleep -> ~25 Hz loop frequency
    done # End of main navigation loop

    # If the loop exits because parking state was triggered, run parking routine here
    if [[ "$is_parking" == true ]]; then
        run_parking_routine
    fi

    log "INFO" "Navigation Controller Loop finished."
}

# --- Parking Routine (Called when parking state is entered) ---
run_parking_routine() {
    log "INFO" "Starting Parking Routine..."
    parking_state="SEARCHING"

    local front_dist left_dist right_dist
    local desired_speed=0
    local desired_angle=90
    local parking_start_time
    parking_start_time=$(date +%s.%3N)

    # --- 1. Search for Parking Spot (if needed - might be known from start/randomization) ---
    # For WRO 2025 Obstacle, the parking lot is in the starting section after 3 laps.
    # Assume the robot needs to navigate towards the general area first (handled by main loop until state change).
    # Now, focus on alignment and parking once in the vicinity.
    parking_state="ALIGNING"

    # --- 2. Align with Parking Space ---
    log "INFO" "Aligning for parking..."
    local align_start_time
    align_start_time=$(date +%s.%3N)
    local align_duration=2000 # milliseconds to spend aligning
    while [[ $(($(date +%s.%3N) - align_start_time)) -lt $align_duration ]]; do
        front_dist=$(read_json_state "$SENSOR_DATA_FILE" "front")
        left_dist=$(read_json_state "$SENSOR_DATA_FILE" "left")
        right_dist=$(read_json_state "$SENSOR_DATA_FILE" "right")

        # Use US sensors to center between potential parking blocks (magenta)
        local align_error
        align_error=$((left_dist - right_dist))
        # Simple proportional control for alignment
        local align_correction
        align_correction=$(echo "0.5 * $align_error" | bc -l) # Adjust gain as needed
        desired_angle=$((90 + align_correction))
        desired_angle=$((desired_angle > 120 ? 120 : (desired_angle < 60 ? 60 : desired_angle))) # Limit steering during parking
        desired_speed=$((MAX_MOTOR_SPEED / 8)) # Very slow during alignment

        echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
        echo "SERVO $desired_angle" > "$COMMAND_QUEUE_FILE"
        sleep 0.05
    done

    # --- 3. Approach Parking Spot ---
    log "INFO" "Approaching parking spot..."
    parking_state="APPROACHING"
    desired_angle=90 # Keep straight
    desired_speed=$((MAX_MOTOR_SPEED / 6)) # Slow forward
    while true; do
        front_dist=$(read_json_state "$SENSOR_DATA_FILE" "front")
        if (( front_dist < PARKING_DISTANCE_THRESHOLD )); then
            log "INFO" "Close to parking spot (US: $front_dist cm). Stopping motor."
            desired_speed=0
            echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
            break
        fi
        echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
        echo "SERVO $desired_angle" > "$COMMAND_QUEUE_FILE"
        sleep 0.05
    done

    # --- 4. Execute Parking Maneuver ---
    log "INFO" "Executing parking maneuver..."
    parking_state="MANEUVERING"
    # Example: Parallel Park (backwards and turn)
    # 1. Reverse slightly
    desired_speed=$(( -MAX_MOTOR_SPEED / 4 ))
    desired_angle=90 # Straight
    echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
    echo "SERVO $desired_angle" > "$COMMAND_QUEUE_FILE"
    sleep 0.8
    # 2. Turn wheels towards the parking space (e.g., if parking on the right, turn right)
    # Assuming parking space is on the right based on typical layout or known start zone.
    # This needs dynamic determination if parking space side varies significantly.
    desired_angle=120 # Turn right wheels
    echo "SERVO $desired_angle" > "$COMMAND_QUEUE_FILE"
    sleep 0.1
    # 3. Reverse more
    echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
    sleep 1.2
    # 4. Turn wheels the other way
    desired_angle=60 # Turn left wheels
    echo "SERVO $desired_angle" > "$COMMAND_QUEUE_FILE"
    sleep 0.1
    # 5. Move forward slightly
    desired_speed=$(( MAX_MOTOR_SPEED / 4 ))
    echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
    sleep 0.5
    # 6. Straighten wheels
    desired_angle=90
    echo "SERVO $desired_angle" > "$COMMAND_QUEUE_FILE"
    sleep 0.1
    # 7. Stop
    desired_speed=0
    echo "MOTOR $desired_speed" > "$COMMAND_QUEUE_FILE"
    log "INFO" "Parking maneuver completed (or attempted)."

    parking_state="FINISHED"
    # Update orchestrator state to FINISHED
    update_json_state "$STATE_FILE" "state" "FINISHED" "string"
    log "INFO" "Parking Routine finished."
}


# --- Cleanup ---
cleanup() {
    log "INFO" "Shutting down Navigation Controller..."
    # Ensure motor is stopped and servo is centered
    echo "MOTOR 0" > "$COMMAND_QUEUE_FILE" 2>/dev/null || true
    echo "SERVO 90" > "$COMMAND_QUEUE_FILE" 2>/dev/null || true
    log "INFO" "Navigation Controller shutdown complete."
}

# --- Signal Traps ---
trap cleanup EXIT
trap 'exit 0' SIGTERM SIGINT

# --- Run Main Loop ---
main_loop "$@"