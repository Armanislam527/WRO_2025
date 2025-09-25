# utils.sh
# Utility functions for the Pi Control Software

log() {
    local level=$1
    shift
    echo "[$(date '+%Y-%m-%d %H:%M:%S.%3N')] [$level] $*" >&2
}

# Function to safely update a JSON state file
update_json_state() {
    local state_file=$1
    local key=$2
    local value=$3
    local type=$4 # "string", "number", "boolean", "null"

    local jq_command
    case $type in
        "string") jq_command=".${key} = \"${value}\"" ;;
        "number") jq_command=".${key} = ${value}" ;;
        "boolean") jq_command=".${key} = ${value}" ;;
        "null") jq_command=".${key} = null" ;;
        *) log "ERROR" "Invalid type '$type' for update_json_state"; return 1 ;;
    esac

    jq "$jq_command" "$state_file" > "${state_file}.tmp" && mv "${state_file}.tmp" "$state_file"
}

# Function to read a value from a JSON state file
read_json_state() {
    local state_file=$1
    local key=$2
    jq -r ".${key}" "$state_file" 2>/dev/null
}

# Function to check if a file exists and is readable
check_file() {
    local file_path=$1
    if [[ ! -r "$file_path" ]]; then
        log "ERROR" "File not found or not readable: $file_path"
        return 1
    fi
    return 0
}