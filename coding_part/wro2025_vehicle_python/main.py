# main.py (Refined and Integrated)
"""Main application entry point for the WRO 2025 Vehicle (Python version)."""

import signal
import sys
import time
import logging
import argparse

# Import our modules
import utils.logger
import config.vehicle_config as cfg
from communication.serial_handler import SerialHandler
from sensors.sensor_data import SensorData
from navigation.mission_state import MissionState, ChallengeType, MissionPhase
from navigation.lap_counter import LapCounter
from navigation.positional_memory import PositionalMemory
# --- Import Vision Modules ---
from vision.frame_buffer import FrameBuffer
from vision.camera_interface import CameraInterface
from vision.image_processor import ImageProcessor
# --- --- ---
from control.path_planner import PathPlanner
from control.navigation_command import NavigationCommand, NavigationAction
from actuators.vehicle_controller import VehicleController

# --- Global State ---
shutdown_requested = False

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    global shutdown_requested
    print("\n[Main] Shutdown signal received (Ctrl+C).")
    shutdown_requested = True

def initialize_system() -> dict:
    """
    Initialize all system components.
    Returns:
        dict: A dictionary containing initialized module instances, or None on failure.
    """
    print("[Main] Initializing system...")
    modules = {}

    try:
        utils.logger.setup_logging()
        logger = logging.getLogger(__name__)
        logger.info("Main application starting.")

        # 1. Serial Handler
        serial_handler = SerialHandler(cfg.SERIAL_PORT, cfg.SERIAL_BAUDRATE)
        if not serial_handler.open():
            logger.error("Failed to open serial port. Exiting.")
            return None
        modules['serial_handler'] = serial_handler
        logger.info("Serial handler initialized.")

        # 2. Mission State
        mission_state = MissionState()
        modules['mission_state'] = mission_state
        logger.info("Mission state initialized.")

        # 3. Lap Counter
        lap_counter = LapCounter()
        if not lap_counter.initialize():
             logger.error("Failed to initialize LapCounter. Exiting.")
             return None
        modules['lap_counter'] = lap_counter
        logger.info("Lap counter initialized.")

        # 4. Positional Memory
        positional_memory = PositionalMemory()
        modules['positional_memory'] = positional_memory
        logger.info("Positional memory initialized.")

        # 5. Path Planner (depends on LapCounter)
        path_planner = PathPlanner()
        if not path_planner.initialize(lap_counter):
            logger.error("Failed to initialize PathPlanner. Exiting.")
            return None
        modules['path_planner'] = path_planner
        logger.info("Path planner initialized.")

        # 6. Vehicle Controller (depends on SerialHandler)
        vehicle_controller = VehicleController(serial_handler)
        modules['vehicle_controller'] = vehicle_controller
        logger.info("Vehicle controller initialized.")

        # 7. Vision System
        frame_buffer = FrameBuffer(maxsize=5) # Reasonable buffer size
        camera_interface = CameraInterface(frame_buffer)
        image_processor = ImageProcessor(frame_buffer)
        
        if not image_processor.initialize():
             logger.error("Failed to initialize ImageProcessor. Exiting.")
             return None

        modules['frame_buffer'] = frame_buffer
        modules['camera_interface'] = camera_interface
        modules['image_processor'] = image_processor
        logger.info("Vision system initialized.")

        logger.info("System initialization complete.")
        return modules

    except Exception as e:
        logging.getLogger(__name__).error(f"Error during system initialization: {e}", exc_info=True)
        return None

def shutdown_system(modules: dict):
    """Shutdown all system components gracefully."""
    logger = logging.getLogger(__name__)
    logger.info("Shutting down system...")
    try:
        # Stop vision components first
        if 'camera_interface' in modules:
            modules['camera_interface'].stop_capture()
        if 'image_processor' in modules:
            modules['image_processor'].stop_processing()

        # Stop serial communication
        if 'serial_handler' in modules:
            modules['serial_handler'].close()

        logger.info("System shutdown complete.")
    except Exception as e:
        logger.error(f"Error during system shutdown: {e}", exc_info=True)

def run_main_loop(modules: dict):
    """
    Main application loop. Coordinates state, data, planning, and control.
    """
    logger = logging.getLogger(__name__)
    logger.info("Entering main application loop.")

    # Get module references
    serial_handler: SerialHandler = modules['serial_handler']
    mission_state: MissionState = modules['mission_state']
    lap_counter: LapCounter = modules['lap_counter']
    positional_memory: PositionalMemory = modules['positional_memory']
    path_planner: PathPlanner = modules['path_planner']
    vehicle_controller: VehicleController = modules['vehicle_controller']
    camera_interface: CameraInterface = modules['camera_interface']
    image_processor: ImageProcessor = modules['image_processor']

    # --- Start Vision System ---
    if not camera_interface.start_capture():
        logger.error("Failed to start camera capture. Vision will be unavailable.")
    else:
         if not image_processor.start_processing():
             logger.error("Failed to start image processing.")
             camera_interface.stop_capture() # Stop capture if processing fails
         else:
             logger.info("Vision system started.")

    # --- Main Loop Timing ---
    loop_rate_hz = 20.0
    loop_period = 1.0 / loop_rate_hz
    last_heartbeat_time = time.time()
    heartbeat_interval = 1.0

    # --- State Flags for Mission Start ---
    initial_data_stored = False
    mission_officially_started = False # After GO ACK

    logger.info(f"Main loop target rate: {loop_rate_hz} Hz")

    # --- Main Loop ---
    while not shutdown_requested:
        loop_start_time = time.time()

        # --- 1. Get Latest Data ---
        latest_sensor_data: SensorData = serial_handler.getLatestSensorData()
        latest_vision_snapshot = image_processor.get_latest_snapshot() # Could be None

        # --- 2. Update State (based on data and time) ---
        
        # --- Handle PRE_START Phase ---
        if mission_state.get_phase() == MissionPhase.PRE_START:
            if serial_handler.hasStartAck():
                 logger.info("[Main] Start button acknowledged by Nano.")
                 mission_state.set_phase(MissionPhase.WAITING_FOR_GO)
                 serial_handler.acknowledgeStartAck()

        # --- Handle WAITING_FOR_GO Phase ---
        elif mission_state.get_phase() == MissionPhase.WAITING_FOR_GO:
            # Send GO command immediately after START_ACK (mirroring C++ logic)
            logger.info("[Main] Sending GO command to Nano.")
            serial_handler.sendGoCommand()
            
            # Assume Open Challenge initially (or determine later)
            mission_state.set_challenge_type(ChallengeType.OPEN_CHALLENGE)
            mission_state.set_phase(MissionPhase.DRIVING_LAPS)
            mission_state.set_current_lap(0)
            mission_officially_started = True # Mark the official start time
            logger.info("[Main] Mission started: Open Challenge, Driving Laps.")

        # --- Handle DRIVING_LAPS, STOPPING_AT_START, FINDING_PARKING, PARKING Phases ---
        elif mission_state.get_phase() in [
            MissionPhase.DRIVING_LAPS, 
            MissionPhase.STOPPING_AT_START, 
            MissionPhase.FINDING_PARKING, 
            MissionPhase.PARKING
        ]:
        
            # --- CRITICAL: Store Initial Data Exactly Once ---
            if mission_officially_started and not initial_data_stored:
                # Store the very first sensor and vision snapshot after GO
                positional_memory.store_initial_position(latest_sensor_data, latest_vision_snapshot)
                # Also store as pre-stop position initially, can be updated later
                positional_memory.store_pre_stop_position(latest_sensor_data, latest_vision_snapshot)
                initial_data_stored = True
                logger.info("[Main] INITIAL SENSOR AND VISION DATA STORED FOR PARKING/STOPPING.")

            # --- Update Lap Counter (placeholder logic, needs refinement) ---
            # A more robust system would use PositionalMemory to detect lap completion.
            # For now, we simulate or use a simple counter increment.
            # lap_counter.update(latest_sensor_data, latest_vision_snapshot)

            # --- Check for Lap Completion (using PositionalMemory) ---
            # This is a key part of making the system robust.
            if initial_data_stored and mission_state.get_current_lap() < 3:
                # Use a high threshold for lap completion to ensure we are really back
                similarity_to_start = positional_memory.compare_with_initial(latest_sensor_data, latest_vision_snapshot)
                lap_completion_threshold = 0.95 # High threshold
                if similarity_to_start > lap_completion_threshold and not lap_counter.is_lap_completed():
                    logger.info(f"[Main] Lap completion detected (Sim: {similarity_to_start:.2f}).")
                    lap_counter.increment_lap() # Sets internal flag

            # --- Process Lap Completion Flag ---
            if lap_counter.is_lap_completed():
                current_lap = lap_counter.get_current_lap()
                mission_state.set_current_lap(current_lap)
                logger.info(f"[Main] Lap {current_lap} acknowledged.")
                
                if current_lap >= 3:
                    if mission_state.is_open_challenge():
                        mission_state.set_phase(MissionPhase.STOPPING_AT_START)
                        logger.info("[Main] 3 laps completed (Open). Preparing to stop in start.")
                    elif mission_state.is_obstacle_challenge():
                        mission_state.set_phase(MissionPhase.FINDING_PARKING)
                        logger.info("[Main] 3 laps completed (Obstacle). Finding parking spot.")
                
                lap_counter.acknowledge_lap_completion() # Clear the flag

            # --- Check for Stopping Condition (Open Challenge) ---
            if mission_state.get_phase() == MissionPhase.STOPPING_AT_START:
                # Use a very high threshold to be sure we are at the exact starting spot
                is_at_pre_stop = positional_memory.is_likely_at_pre_stop_position(
                    latest_sensor_data, latest_vision_snapshot, threshold=0.98
                )
                if is_at_pre_stop:
                    logger.info("[Main] Likely at pre-stop position. Executing final stop.")
                    nav_command_stop = NavigationCommand(NavigationAction.STOP_IN_START, 0.0)
                    vehicle_controller.execute_command(nav_command_stop)
                    mission_state.set_phase(MissionPhase.COMPLETED)
                    logger.info("[Main] Vehicle stopped. Mission COMPLETED (Open).")

            # --- Check for Parking Initiation (Obstacle Challenge) ---
            # This check can be refined. For now, PathPlanner handles finding the spot.
            # if (mission_state.get_phase() == MissionPhase.FINDING_PARKING and
            #     positional_memory.is_likely_at_parking_entrance(latest_sensor_data, latest_vision_snapshot, threshold=0.85)):
            #     mission_state.set_phase(MissionPhase.PARKING)
            #     logger.info("[Main] Parking entrance confirmed. Initiating parking sequence.")
            #     vehicle_controller.start_parking_sequence() # Or let PathPlanner command do it

        # --- 3. Plan (only if not completed/failed) ---
        nav_command = NavigationCommand() # Default
        if mission_state.get_phase() not in [MissionPhase.COMPLETED, MissionPhase.EMERGENCY_STOPPED]:
            nav_command = path_planner.plan(latest_sensor_data, latest_vision_snapshot, mission_state)

        # --- 4. Act ---
        vehicle_controller.execute_command(nav_command)

        # --- 5. Housekeeping ---
        current_time = time.time()
        if current_time - last_heartbeat_time > heartbeat_interval:
            if mission_state.is_driving_phase() and not vehicle_controller.is_maneuver_active():
                serial_handler.sendHeartbeat()
            last_heartbeat_time = current_time

        # --- 6. Maintain Loop Rate ---
        loop_end_time = time.time()
        execution_time = loop_end_time - loop_start_time
        sleep_time = loop_period - execution_time
        if sleep_time > 0:
            time.sleep(sleep_time)

    logger.info("Main loop exited.")

def main():
    """Entry point of the application."""
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("[Main] WRO 2025 Vehicle Controller (Python) Starting...")
    modules = initialize_system()
    # if not hasattr(modules.get('serial_handler'), '_test_sent'):
    #     print("[TEST] Sending test motor speed (50) and servo angle (90)")
    #     serial_handler.sendMotorSpeedCommand(50)
    #     serial_handler.sendServoAngleCommand(90)
    #     modules['serial_handler']._test_sent = True
    if modules:
        try:
            run_main_loop(modules)
        except Exception as e:
            logging.getLogger(__name__).error(f"Unhandled exception in main loop: {e}", exc_info=True)
        finally:
            shutdown_system(modules)
    else:
        print("[Main] System initialization failed. Exiting.")
        sys.exit(1)

    print("[Main] WRO 2025 Vehicle Controller Shutdown Complete.")
    sys.exit(0)

if __name__ == "__main__":
    main()
