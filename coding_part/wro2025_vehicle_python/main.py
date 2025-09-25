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
from vision.frame_buffer import FrameBuffer
from vision.camera_interface import CameraInterface
from vision.image_processor import ImageProcessor
from control.path_planner import PathPlanner
from control.navigation_command import NavigationCommand, NavigationAction
from actuators.vehicle_controller import VehicleController

# Global state
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
        frame_buffer = FrameBuffer(maxsize=5)
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
        if 'camera_interface' in modules:
            modules['camera_interface'].stop_capture()
        if 'image_processor' in modules:
            modules['image_processor'].stop_processing()
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

    # Start vision system
    if not camera_interface.start_capture():
        logger.error("Failed to start camera capture. Using sensor data only.")
    else:
        if not image_processor.start_processing():
            logger.error("Failed to start image processing.")
            camera_interface.stop_capture()
        else:
            logger.info("Vision system started.")

    # Main loop timing
    loop_rate_hz = 10.0  # Reduced for Pi Zero 2W
    loop_period = 1.0 / loop_rate_hz
    last_heartbeat_time = time.time()
    heartbeat_interval = 1.0
    start_time = time.time()
    start_timeout = 10.0  # Timeout for StartAck

    # State flags
    initial_data_stored = False
    mission_officially_started = False
    iteration_count = 0

    logger.info(f"Main loop target rate: {loop_rate_hz} Hz")

    while not shutdown_requested:
        try:
            iteration_count += 1
            loop_start_time = time.time()
            logger.debug(f"[Loop {iteration_count}] Phase: {mission_state.get_phase()}")

            # Send heartbeat to prevent Nano watchdog
            if time.time() - last_heartbeat_time > heartbeat_interval:
                serial_handler.sendHeartbeat()
                last_heartbeat_time = time.time()
                logger.debug("Heartbeat sent.")

            # Get latest data
            latest_sensor_data: SensorData = serial_handler.getLatestSensorData()
            latest_vision_snapshot = image_processor.get_latest_snapshot()
            if not latest_vision_snapshot or not latest_vision_snapshot.edges_searched:
                logger.warning("No valid vision data; using sensor data only.")
                latest_vision_snapshot = VisionSnapshot()

            logger.debug(f"[Loop {iteration_count}] Sensor F={latest_sensor_data.us_data.front_distance:.1f}cm")
            if latest_vision_snapshot:
                logger.debug(f"[Loop {iteration_count}] Vision: LSign={latest_vision_snapshot.primary_left_sign}, RSign={latest_vision_snapshot.primary_right_sign}")

            # Mission state logic
            if mission_state.get_phase() == MissionPhase.PRE_START:
                if serial_handler.hasStartAck():
                    logger.info("StartAck received. Sending GO.")
                    serial_handler.sendGoCommand()
                    mission_state.set_phase(MissionPhase.WAITING_FOR_GO)
                    mission_officially_started = True
                elif time.time() - start_time > start_timeout:
                    logger.warning("No StartAck; forcing GO for testing.")
                    serial_handler.sendGoCommand()
                    mission_state.set_phase(MissionPhase.WAITING_FOR_GO)
                    mission_officially_started = True
            elif mission_state.get_phase() == MissionPhase.WAITING_FOR_GO:
                if serial_handler.hasGoAck():
                    logger.info("GO Ack received. Starting driving.")
                    mission_state.set_phase(MissionPhase.DRIVING_LAPS)
            elif mission_state.get_phase() == MissionPhase.DRIVING_LAPS:
                # Store initial data
                if not initial_data_stored:
                    positional_memory.store_initial_data(latest_sensor_data, latest_vision_snapshot)
                    initial_data_stored = True

                # Update navigation
                lap_counter.update(latest_vision_snapshot, latest_sensor_data)
                path_plan = path_planner.plan_path(
                    latest_vision_snapshot,
                    latest_sensor_data,
                    mission_state.get_challenge_type(),
                    lap_counter.get_lap_count()
                )
                vehicle_controller.execute_command(path_plan)

            # Maintain loop rate
            elapsed = time.time() - loop_start_time
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

        except Exception as e:
            logger.error(f"Loop error: {e}", exc_info=True)

    logger.info("Main loop exited.")
    shutdown_system(modules)

def main():
    """Main entry point."""
    print("[Main] WRO 2025 Vehicle Controller (Python) Starting...")
    signal.signal(signal.SIGINT, signal_handler)
    modules = initialize_system()
    if modules is None:
        print("[Main] Initialization failed. Exiting.")
        sys.exit(1)
    run_main_loop(modules)
    print("[Main] WRO 2025 Vehicle Controller Shutdown Complete.")

if __name__ == "__main__":
    main()