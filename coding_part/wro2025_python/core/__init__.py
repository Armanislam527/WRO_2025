"""
Core package for WRO 2025 vehicle system
"""

from .process_manager import process_manager, ProcessManager
from .shared_memory import shared_memory, SharedMemoryManager
from .state_machine import state_machine, CompetitionStateMachine

__all__ = [
    'process_manager', 'ProcessManager',
    'shared_memory', 'SharedMemoryManager', 
    'state_machine', 'CompetitionStateMachine'
]