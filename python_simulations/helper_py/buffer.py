"""
Buffer management for OpenGL using PyOpenGL.
Uses PySide6's QObject for lifecycle management only.
"""

from typing import List, Optional, TypeVar, Generic
import numpy as np
from PySide6.QtCore import QObject
from OpenGL.GL import *

T = TypeVar('T')


class Buffer(Generic[T], QObject):
    """
    OpenGL buffer wrapper class using Qt's OpenGL functions.
    
    This class manages OpenGL buffer objects (VBOs, EBOs) for storing vertex data.
    """
    
    def __init__(self, 
                 target: int,
                 usage: int,
                 max_items: Optional[int] = None,
                 data: Optional[List[T]] = None,
                 parent: Optional[QObject] = None):
        """
        Initialize a new Buffer.
        
        Args:
            target: Buffer target (GL_ARRAY_BUFFER or GL_ELEMENT_ARRAY_BUFFER)
            usage: Buffer usage hint (GL_STATIC_DRAW, GL_DYNAMIC_DRAW, GL_STREAM_DRAW)
            max_items: Maximum number of items (required if data is None)
            data: Initial data to load (optional)
            parent: Parent QObject for memory management
        """
        super().__init__(parent)
        
        self._target = target
        self._usage = usage
        
        # Generate buffer using PyOpenGL
        self._buffer = glGenBuffers(1)
        
        # Determine data type based on buffer target
        if target == GL_ELEMENT_ARRAY_BUFFER:
            self._dtype = np.uint16  # Index buffers use unsigned short
        else:
            self._dtype = np.float32  # Vertex buffers use float
        
        if data is not None:
            self._size = len(data)
            self._capacity = len(data)
            glBindBuffer(target, self._buffer)
            np_data = np.array(data, dtype=self._dtype)
            if self._dtype == np.float32:
                np_data = np_data.flatten()
            glBufferData(target, np_data.nbytes, np_data, usage)
            glBindBuffer(target, 0)
        else:
            if max_items is None:
                raise ValueError("Either max_items or data must be provided")
            self._size = 0
            self._capacity = max_items
            glBindBuffer(target, self._buffer)
            item_size = np.dtype(self._dtype).itemsize
            glBufferData(target, max_items * item_size, None, usage)
            glBindBuffer(target, 0)
    
    def __del__(self):
        """Cleanup buffer on deletion."""
        if hasattr(self, '_buffer') and self._buffer != 0:
            try:
                glDeleteBuffers(1, [self._buffer])
            except:
                pass
    
    def size(self) -> int:
        """Return the current number of items in the buffer."""
        return self._size
    
    def capacity(self) -> int:
        """Return the maximum capacity of the buffer."""
        return self._capacity
    
    def bind(self):
        """Bind the buffer to its target."""
        glBindBuffer(self._target, self._buffer)
    
    def unbind(self):
        """Unbind the buffer from its target."""
        glBindBuffer(self._target, 0)
    
    def clear(self):
        """Clear the buffer (resets size to 0)."""
        self._size = 0
    
    def load(self, data: List[T]):
        """
        Load data into the buffer, replacing existing contents.
        
        Args:
            data: List of data items to load
        """
        glBindBuffer(self._target, self._buffer)
        np_data = np.array(data, dtype=self._dtype)
        if self._dtype == np.float32:
            np_data = np_data.flatten()
        glBufferSubData(self._target, 0, np_data.nbytes, np_data)
        glBindBuffer(self._target, 0)
        self._size = len(data)
    
    def append(self, data: List[T]):
        """
        Append data to the buffer.
        
        Args:
            data: List of data items to append
        """
        if self._size + len(data) > self._capacity:
            raise ValueError(f"Cannot append {len(data)} items: buffer capacity ({self._capacity}) would be exceeded")
        
        glBindBuffer(self._target, self._buffer)
        np_data = np.array(data, dtype=self._dtype)
        if self._dtype == np.float32:
            np_data = np_data.flatten()
        offset = self._size * np_data.itemsize
        glBufferSubData(self._target, offset, np_data.nbytes, np_data)
        glBindBuffer(self._target, 0)
        self._size += len(data)
    
    def load_region(self, start: int, size: int, data: List[T]):
        """
        Load data into a specific region of the buffer.
        
        Args:
            start: Starting index
            size: Number of items to load
            data: List of data items (must have at least 'size' elements)
        """
        if len(data) < size:
            raise ValueError(f"Data list has {len(data)} elements but {size} are required")
        
        glBindBuffer(self._target, self._buffer)
        np_data = np.array(data[:size], dtype=self._dtype)
        if self._dtype == np.float32:
            np_data = np_data.flatten()
        offset = start * np_data.itemsize
        glBufferSubData(self._target, offset, np_data.nbytes, np_data)
        glBindBuffer(self._target, 0)
    
    def buffer_id(self) -> int:
        """Return the OpenGL buffer ID."""
        return self._buffer
    
    def __call__(self) -> int:
        """Return the OpenGL buffer ID (allows using buffer() syntax)."""
        return self.buffer_id()


def ArrayBuffer(usage: int, max_items: Optional[int] = None,
                data: Optional[List] = None, parent: Optional[QObject] = None) -> Buffer:
    """
    Create an array buffer (VBO).
    
    Args:
        usage: Buffer usage hint (GL_STATIC_DRAW, GL_DYNAMIC_DRAW, GL_STREAM_DRAW)
        max_items: Maximum number of items (required if data is None)
        data: Initial data to load (optional)
        parent: Parent QObject for memory management
    
    Returns:
        A Buffer configured as an array buffer
    """
    return Buffer(GL_ARRAY_BUFFER, usage, max_items, data, parent)


def ElementArrayBuffer(usage: int, max_items: Optional[int] = None,
                       data: Optional[List] = None, parent: Optional[QObject] = None) -> Buffer:
    """
    Create an element array buffer (EBO/IBO).
    
    Args:
        usage: Buffer usage hint (GL_STATIC_DRAW, GL_DYNAMIC_DRAW, GL_STREAM_DRAW)
        max_items: Maximum number of items (required if data is None)
        data: Initial data to load (optional)
        parent: Parent QObject for memory management
    
    Returns:
        A Buffer configured as an element array buffer
    """
    return Buffer(GL_ELEMENT_ARRAY_BUFFER, usage, max_items, data, parent)
