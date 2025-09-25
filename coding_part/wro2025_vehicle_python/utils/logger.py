# utils/logger.py

"""Centralized logging configuration."""

import logging
import config.vehicle_config as cfg

def setup_logging():
    """Configure the logging module based on config."""
    level_name = cfg.LOG_LEVEL.upper()  # Convert to uppercase for consistency
    level = getattr(logging, level_name, logging.DEBUG)  # Default to DEBUG if invalid
    if level_name not in ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']:
        print(f"[Logger Warning] Invalid LOG_LEVEL '{cfg.LOG_LEVEL}' in config. Defaulting to DEBUG.")
        level = logging.DEBUG

    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    # logger = logging.getLogger(__name__)
    # logger.info(f"Logging configured with level: {cfg.LOG_LEVEL}")
    print(f"[Logger] Logging initialized with level: {logging.getLevelName(level)} (level_name: {level_name})") # Use print before logger is fully set up

# Convenience functions (optional, if you prefer module-level calls)
def debug(msg): logging.getLogger().debug(msg)
def info(msg): logging.getLogger().info(msg)
def warning(msg): logging.getLogger().warning(msg)
def error(msg): logging.getLogger().error(msg)

# Example usage in other modules:
# import utils.logger
# utils.logger.setup_logging() # Call once in main
# logger = logging.getLogger(__name__)
# logger.info("This is an info message")
