# vision/frame_buffer.py (Efficiency Check)
"""Thread-safe frame buffer (queue) for inter-thread communication."""

import queue
import logging
from typing import Optional

logger = logging.getLogger(__name__)

class FrameBuffer:
    """
    A thread-safe queue to hold image frames (JPEG bytes).
    """

    def __init__(self, maxsize: int = 5): # Reduced maxsize for lower memory on Pi Zero
        """
        Args:
            maxsize (int): Maximum number of frames. Oldest discarded if full.
        """
        # Use a small maxsize to prevent memory buildup and force newer frames
        self._queue = queue.Queue(maxsize=maxsize)
        self.maxsize = maxsize
        logger.debug(f"FrameBuffer initialized with maxsize {maxsize}.")

    def put(self, frame_data: bytes):
        """Put a frame into the buffer, discarding oldest if full."""
        try:
            self._queue.put_nowait(frame_data)
        except queue.Full:
            try:
                self._queue.get_nowait() # Discard oldest
                self._queue.put_nowait(frame_data) # Add new
                # logger.debug("FrameBuffer: Discarded oldest frame.")
            except (queue.Empty, queue.Full):
                logger.warning("FrameBuffer: Unexpected state during put.")

    def get(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Get a frame from the buffer."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def empty(self) -> bool:
        return self._queue.empty()

    def qsize(self) -> int:
        return self._queue.qsize()

    def clear(self):
        """Clear all frames."""
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
        # logger.debug("FrameBuffer cleared.")
