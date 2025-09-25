#!/bin/bash

# hardware_interface.sh
# Handles communication with the Arduino Nano using the structured packet protocol.
# Designed for a headless Raspberry Pi OS.
# *** CORRECTED COMMAND IDs based on nano_code.txt ***

set -e # Exit on any error

# --- Configuration (can be sourced from a config file) ---
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"  # Default or from environment/config
BAUD_RATE="${BAUD_RATE:-115200}"
SERIAL_DEVICE="$SERIAL_PORT"
HEARTBEAT_INTERVAL="${HEARTBEAT_INTERVAL:-200}" # milliseconds (must be less than Nano's watchdog timeout)
COMM_TIMEOUT="${COMM_TIMEOUT:-1000}" # milliseconds for initial packet read attempt
PACKET_MAX_LENGTH=32
MAX_PACKET_SIZE=$((4 + PACKET_MAX_LENGTH + 1)) # Start(2) + Cmd(1) + Len(1) + Data + Checksum(1)

# --- State Variables (Managed by this script or shared files) ---
sensor_data_file="${SENSOR_DATA_FILE:-/tmp/nano_sensor_data.json}" # Path to shared JSON file for sensor data
command_queue_file="${COMMAND_QUEUE_FILE:-/tmp/pi_commands.fifo}" # FIFO for receiving commands from other scripts
state_file="${STATE_FILE:-/tmp/pi_state.json}" # Path to shared JSON file for overall state (e.g., started, go_received, error)

# --- Internal Variables ---
start_byte1=0x55
start_byte2=0xAA
# Command IDs (from serial_protocol.h on Nano - CORRECTED)
CMD_SET_MOTOR_SPEED=0x01
CMD_SET_SERVO_ANGLE=0x02
CMD_EMERGENCY_STOP=0x03
CMD_HEARTBEAT=0x04
CMD_REQUEST_SENSOR_DATA=0x05
CMD_PI_GO_SIGNAL=0x06 # *** CORRECTED ID ***
# Command IDs from Nano to Pi
CMD_START_ACK=0x11 # *** CORRECTED ID ***
CMD_ERROR=0x12
CMD_ALL_SENSOR_DATA_COMPACT=0x13

# --- Logging Function ---
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S.%3N')] [HardwareInterface] $*" >&2
}

# --- Checksum Calculation (XOR) ---
calculate_checksum() {
    local -n data_array=$1
    local checksum=0
    for byte in "${data_array[@]}"; do
        checksum=$((checksum ^ byte))
    done
    printf "%d" "$checksum" # Return integer checksum
}

# --- Send Packet to Nano ---
send_packet() {
    local command_id=$1
    local data_len=$2
    local -n data_bytes=$3 # Use nameref to access the array passed by name

    # Calculate checksum
    local checksum_input=()
    checksum_input+=("$command_id" "$data_len")
    checksum_input+=("${data_bytes[@]}")
    local checksum=$(calculate_checksum checksum_input)

    # Prepare packet bytes (start bytes are already hex literals like 0x55/0xAA)
    local packet_bytes=()
    packet_bytes+=("$start_byte1" "$start_byte2" "$command_id" "$data_len")
    packet_bytes+=("${data_bytes[@]}")
    packet_bytes+=("$checksum")

    # Convert bytes to binary string for sending
    local binary_packet=""
    for b in "${packet_bytes[@]}"; do
        binary_packet+=$(printf "\\x%02X" "$((b & 0xFF))") # Ensure byte is 0-255
    done

    # Send packet via serial using echo and redirection
    if [[ -w "$SERIAL_DEVICE" ]]; then
        echo -ne "$binary_packet" > "$SERIAL_DEVICE"
        # log "Sent packet CMD: 0x$(printf "%02X" "$command_id") LEN: $data_len CHK: 0x$(printf "%02X" "$checksum")"
    else
        log "ERROR: Cannot write to serial device $SERIAL_DEVICE"
        return 1
    fi
}

# --- Receive Packet from Nano (Bash Implementation) ---
receive_packet() {
    local -n packet_out=$1
    local timeout_ms=$2
    local start_time=$(($(date +%s%3N)))

    # Reset packet buffer
    unset packet_out
    declare -gA packet_out # Declare as global associative array

    local state="WAITING_START1"
    local byte
    local pos=0
    local cmd_id len data_len chk calc_chk
    local packet_buffer=()

    # Configure serial port settings for raw binary input
    stty -F "$SERIAL_DEVICE" "$BAUD_RATE" raw -echo -icanon -icrnl -ixon -ixoff

    # Set initial timeout for the read operation
    local remaining_time=$((timeout_ms))
    if [[ $remaining_time -le 0 ]]; then
        log "ERROR: Timeout before starting read"
        return 1
    fi

    # Calculate deadline
    local deadline=$((start_time + timeout_ms))

    while true; do
        # Check for timeout
        local current_time=$(($(date +%s%3N)))
        if (( current_time > deadline )); then
            log "INFO: Receive timeout after $timeout_ms ms"
            return 1
        fi

        # Calculate remaining time for read
        local time_left=$((deadline - current_time))
        # Use 'dd' with a short timeout to avoid blocking indefinitely
        # A very short timeout (e.g., 0.01s = 10ms) allows checking the deadline loop
        local read_timeout_s=0.01
        local bytes_to_read=1
        local temp_file=$(mktemp)

        # Attempt to read one byte with a short timeout
        # Use 'dd' to read raw binary data
        timeout $read_timeout_s dd bs=1 count=$bytes_to_read if="$SERIAL_DEVICE" of="$temp_file" 2>/dev/null
        local dd_exit_code=$?

        if [[ $dd_exit_code -eq 0 ]]; then
            # Successfully read a byte
            if [[ -s "$temp_file" ]]; then # Check if file has content
                byte=$(od -An -tu1 -N1 "$temp_file" | xargs) # Read as unsigned integer
                rm -f "$temp_file"
                # log "Read byte: 0x$(printf "%02X" "$byte") State: $state Pos: $pos"

                case $state in
                    "WAITING_START1")
                        if [[ $byte -eq 0x55 ]]; then
                            state="WAITING_START2"
                            pos=1
                            packet_buffer=("$byte")
                        fi
                        ;;
                    "WAITING_START2")
                        if [[ $byte -eq 0xAA ]]; then
                            state="WAITING_CMD"
                            pos=2
                            packet_buffer+=("$byte")
                        else
                            state="WAITING_START1" # Reset, wasn't a valid start
                            pos=0
                            packet_buffer=()
                        fi
                        ;;
                    "WAITING_CMD")
                        cmd_id=$byte
                        state="WAITING_LEN"
                        pos=3
                        packet_buffer+=("$byte")
                        ;;
                    "WAITING_LEN")
                        len=$byte
                        data_len=$len
                        state="WAITING_DATA"
                        pos=4
                        packet_buffer+=("$byte")
                        if [[ $data_len -eq 0 ]]; then
                            state="WAITING_CHK"
                        fi
                        ;;
                    "WAITING_DATA")
                        packet_buffer+=("$byte")
                        pos=$((pos + 1))
                        if [[ $pos -eq $((4 + data_len)) ]]; then
                            state="WAITING_CHK"
                        fi
                        ;;
                    "WAITING_CHK")
                        chk=$byte
                        packet_buffer+=("$byte")
                        pos=$((pos + 1))
                        local expected_len=$((4 + data_len + 1))

                        if [[ $pos -eq $expected_len ]]; then
                            # Packet received completely
                            # Calculate checksum for data section (cmd_id, len, data)
                            local calc_checksum_input=("$cmd_id" "$len")
                            local data_section=("${packet_buffer[@]:4:$data_len}")
                            calc_checksum_input+=("${data_section[@]}")
                            calc_chk=$(calculate_checksum calc_checksum_input)

                            if [[ $chk -eq $calc_chk ]]; then
                                packet_out[command]=$cmd_id
                                packet_out[length]=$data_len
                                packet_out[data]=$(printf '%s ' "${data_section[@]}") # Store as space-separated string
                                # packet_out[raw_bytes]=$(printf '%s ' "${packet_buffer[@]}") # Optional: store raw bytes
                                log "Received valid packet CMD: 0x$(printf "%02X" "$cmd_id") LEN: $data_len CHK: 0x$(printf "%02X" "$chk")"
                                return 0 # Success
                            else
                                log "Checksum mismatch: received 0x$(printf "%02X" "$chk"), calculated 0x$(printf "%02X" "$calc_chk")"
                                # Reset state and buffer to look for next packet
                                state="WAITING_START1"
                                pos=0
                                packet_buffer=()
                            fi
                        else
                            log "Incorrect final length. Expected: $expected_len, Got: $pos"
                            # Reset state and buffer to look for next packet
                            state="WAITING_START1"
                            pos=0
                            packet_buffer=()
                        fi
                        ;;
                esac
            else
                # dd succeeded but no data was read, continue loop to check timeout
                rm -f "$temp_file"
                continue
            fi
        elif [[ $dd_exit_code -eq 124 ]]; then
            # Timeout occurred in 'dd', continue loop to check overall timeout
            rm -f "$temp_file"
            continue
        else
            # Other error in 'dd'
            log "ERROR: 'dd' failed with exit code $dd_exit_code"
            rm -f "$temp_file"
            return 1
        fi
    done

    # Should not reach here under normal circumstances due to timeout checks
    log "ERROR: Unexpected exit from receive loop"
    return 1
}


# --- Main Communication Loop ---
main_loop() {
    log "Starting Hardware Interface Loop on $SERIAL_DEVICE..."

    # Ensure serial port is accessible
    if [[ ! -c "$SERIAL_DEVICE" ]]; then
        log "ERROR: Serial device $SERIAL_DEVICE does not exist or is not a character device."
        exit 1
    fi

    # Configure serial port (done once at the start of the loop)
    stty -F "$SERIAL_DEVICE" "$BAUD_RATE" raw -echo -icanon -icrnl -ixon -ixoff

    # Create command queue FIFO if it doesn't exist
    if [[ ! -p "$command_queue_file" ]]; then
        log "Creating command queue FIFO: $command_queue_file"
        mkfifo "$command_queue_file"
    fi

    # Initialize or repair state file to ensure a valid 'state' key
    if [[ ! -f "$state_file" ]]; then
        log "Initializing state file: $state_file"
        echo '{"state": "IDLE", "started": false, "go_received": false, "error": false}' > "$state_file"
    else
        # If 'state' key is missing or null, set to IDLE
        current_state=$(jq -r '.state // empty' "$state_file" 2>/dev/null || echo "")
        if [[ -z "$current_state" || "$current_state" == "null" ]]; then
            log "Repairing state file: adding default 'IDLE' state"
            jq '.state = "IDLE" | .started = (.started // false) | .go_received = (.go_received // false) | .error = (.error // false)' "$state_file" > "$state_file.tmp" && mv "$state_file.tmp" "$state_file"
        fi
    fi

    # Initialize sensor data file
    log "Initializing sensor data file: $sensor_data_file"
    echo '{"front": 0, "right": 0, "back": 0, "left": 0, "accelX": 0.0, "accelY": 0.0, "accelZ": 0.0, "gyroX": 0.0, "gyroY": 0.0, "gyroZ": 0.0}' > "$sensor_data_file"

    local last_heartbeat_time=$(($(date +%s%3N)))
    local packet=()
    local command_to_send=() # Dummy array for send_packet
    local cmd_type cmd_value

    log "Hardware Interface Loop initialized. Waiting for START/GO signal."

    while true; do
        local current_time=$(($(date +%s%3N)))

        # --- 1. Send Heartbeat (if interval elapsed) ---
        if (( current_time - last_heartbeat_time >= HEARTBEAT_INTERVAL )); then
            send_packet $CMD_HEARTBEAT 0 command_to_send # Empty data array
            last_heartbeat_time=$current_time
        fi

        # --- 2. Receive Packet from Nano (Non-blocking check using short timeout) ---
        # Use a short timeout here to allow checking the command queue frequently
        if receive_packet packet 10; then # 10ms timeout for single packet attempt
            local received_cmd=${packet[command]}
            local received_data_str="${packet[data]}"
            local received_data_bytes=()
            if [[ -n "$received_data_str" ]]; then
                received_data_bytes=($received_data_str)
            fi

            case $received_cmd in
                $CMD_START_ACK) # *** CORRECTED CASE ***
                    log "Received START ACK from Nano."
                    # Update state: Started acknowledged by Nano
                    jq '.started = true' "$state_file" > "$state_file.tmp" && mv "$state_file.tmp" "$state_file"
                    ;;
                $CMD_ALL_SENSOR_DATA_COMPACT)
                    # Parse CompactSensorData (20 bytes: 4x uint16, 6x int16)
                    # Raw data bytes are in received_data_bytes (should be 20 elements)
                    if [[ ${#received_data_bytes[@]} -eq 20 ]]; then
                        # Combine bytes for each field (assuming little-endian, adjust if necessary)
                        # For signed 16-bit values (IMU), Bash handles 32-bit signed int well enough for this.
                        # The bit shifting automatically handles sign extension for int16_t.
                        local front_dist=$(( (received_data_bytes[1] << 8) | received_data_bytes[0] ))
                        local right_dist=$(( (received_data_bytes[3] << 8) | received_data_bytes[2] ))
                        local back_dist=$(( (received_data_bytes[5] << 8) | received_data_bytes[4] ))
                        local left_dist=$(( (received_data_bytes[7] << 8) | received_data_bytes[6] ))
                        local accel_x=$(( (received_data_bytes[9] << 8) | received_data_bytes[8] ))
                        local accel_y=$(( (received_data_bytes[11] << 8) | received_data_bytes[10] ))
                        local accel_z=$(( (received_data_bytes[13] << 8) | received_data_bytes[12] ))
                        local gyro_x=$(( (received_data_bytes[15] << 8) | received_data_bytes[14] ))
                        local gyro_y=$(( (received_data_bytes[17] << 8) | received_data_bytes[16] ))
                        local gyro_z=$(( (received_data_bytes[19] << 8) | received_data_bytes[18] ))

                        # Update sensor data JSON file
                        jq -n --argjson f "$front_dist" --argjson r "$right_dist" --argjson b "$back_dist" --argjson l "$left_dist" \
                           --argjson ax "$accel_x" --argjson ay "$accel_y" --argjson az "$accel_z" \
                           --argjson gx "$gyro_x" --argjson gy "$gyro_y" --argjson gz "$gyro_z" \
                           '{front: $f, right: $r, back: $b, left: $l, accelX: $ax, accelY: $ay, accelZ: $az, gyroX: $gx, gyroY: $gy, gyroZ: $gz}' > "$sensor_data_file"
                        # log "Updated sensor data F:$front_dist R:$right_dist B:$back_dist L:$left_dist"
                    else
                        log "ERROR: Compact sensor data packet has wrong length: ${#received_data_bytes[@]}, expected 20."
                        # Potentially trigger error state
                        jq '.error = true' "$state_file" > "$state_file.tmp" && mv "$state_file.tmp" "$state_file"
                    fi
                    ;;
                $CMD_ERROR)
                    log "Received ERROR packet from Nano (Code: 0x$(printf "%02X" "${received_data_bytes[0]:-0}"))."
                    jq '.error = true' "$state_file" > "$state_file.tmp" && mv "$state_file.tmp" "$state_file"
                    # Potentially trigger E-Stop logic in main orchestrator
                    ;;
                *)
                    log "Received unknown command ID: 0x$(printf "%02X" "$received_cmd")"
                    ;;
            esac
            unset packet # Clear packet for next iteration
        fi # End of receive packet attempt

        # --- 3. Send Commands from Queue (if any) ---
        # Use 'read' with a very short timeout (0.001s) to check the FIFO non-blockingly
        if read -t 0.001 cmd_type cmd_value < "$command_queue_file" 2>/dev/null; then
            case $cmd_type in
                "MOTOR")
                    # Scale motor speed (-255 to 255) to byte (-128 to 127) for Nano protocol
                    local scaled_speed
                    # Ensure cmd_value is within range
                    cmd_value=$((cmd_value > 255 ? 255 : (cmd_value < -255 ? -255 : cmd_value)))
                    scaled_speed=$(( (cmd_value * 127) / 255 ))
                    # Ensure scaled_speed is within byte range after mapping
                    scaled_speed=$((scaled_speed > 127 ? 127 : (scaled_speed < -128 ? -128 : scaled_speed)))
                    # Convert signed int to unsigned byte representation for transmission
                    # Bash handles the conversion correctly for the packet.
                    local cmd_data=($((scaled_speed & 0xFF)))
                    send_packet $CMD_SET_MOTOR_SPEED 1 cmd_data
                    ;;
                "SERVO")
                    # Constrain servo angle (0-180) and convert to byte (0-255)
                    local angle_byte
                    angle_byte=$((cmd_value > 180 ? 180 : (cmd_value < 0 ? 0 : cmd_value)))
                    angle_byte=$((angle_byte & 0xFF)) # Ensure it's a single byte
                    local cmd_data=($angle_byte)
                    send_packet $CMD_SET_SERVO_ANGLE 1 cmd_data
                    ;;
                "GO") # *** CORRECTED CASE ***
                    # Send GO signal to Nano to start main operational loop
                    send_packet $CMD_PI_GO_SIGNAL 0 command_to_send # Empty data array
                    jq '.go_received = true' "$state_file" > "$state_file.tmp" && mv "$state_file.tmp" "$state_file"
                    log "Sent GO signal to Nano."
                    ;;
                "STOP")
                    # Send E-Stop command
                    send_packet $CMD_EMERGENCY_STOP 0 command_to_send # Empty data array
                    log "Sent E-Stop command to Nano."
                    ;;
                *)
                    log "Unknown command type from queue: $cmd_type Value: $cmd_value"
                    ;;
            esac
        fi # End of command queue check

        # --- 4. Brief Pause to Prevent Busy-Waiting ---
        # Adjust sleep time based on desired loop frequency. 10ms = 100Hz should be sufficient.
        sleep 0.01 # 10ms sleep
    done # End of main loop

    # Cleanup happens via trap
}

# --- Cleanup Function ---
cleanup() {
    log "Shutting down Hardware Interface..."
    # Send emergency stop before exiting (try to do this gracefully)
    local dummy_cmd=()
    if [[ -w "$SERIAL_DEVICE" ]]; then
        send_packet $CMD_EMERGENCY_STOP 0 dummy_cmd 2>/dev/null || true
    fi
    # Close serial port descriptors if opened via exec (not used here, but good practice)
    # exec 3<&- 2>/dev/null || true
    # exec 3>&- 2>/dev/null || true
    # Note: FIFO is not removed here as other processes might still be using it.
    # It should be handled by the main orchestrator or system startup script.
    log "Hardware Interface shutdown complete."
}

# --- Signal Traps for Graceful Shutdown ---
trap cleanup EXIT
trap 'exit 0' SIGTERM SIGINT

# --- Run Main Loop ---
main_loop "$@"