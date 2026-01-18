"""
Texture management for OpenGL using PyOpenGL.
Uses PySide6's QObject for lifecycle management only.
"""

from typing import Optional
import numpy as np
from PySide6.QtCore import QObject
from OpenGL.GL import *


class Texture2D(QObject):
    """
    OpenGL 2D texture wrapper class using Qt's OpenGL functions.
    
    This class manages OpenGL 2D texture objects for storing image data.
    """
    
    class Options:
        """Options for texture creation."""
        
        def __init__(self):
            self.param_min_filter = GL_LINEAR
            self.param_mag_filter = GL_LINEAR
            self.param_wrap_s = GL_MIRRORED_REPEAT
            self.param_wrap_t = GL_MIRRORED_REPEAT
    
    def __init__(self,
                 internal_format: int,
                 format: int,
                 data_type: int,
                 width: int,
                 height: int,
                 parent: Optional[QObject] = None,
                 data: Optional[bytes] = None,
                 opts: Optional[Options] = None):
        """
        Initialize a new 2D texture.
        
        Args:
            internal_format: Internal format of the texture (e.g., GL_RGB, GL_RGBA)
            format: Format of the pixel data (e.g., GL_RGB, GL_RGBA)
            data_type: Data type of the pixel data (e.g., GL_UNSIGNED_BYTE)
            width: Width of the texture in pixels
            height: Height of the texture in pixels
            parent: Parent QObject for memory management
            data: Initial texture data (optional)
            opts: Texture options (optional)
        """
        super().__init__(parent)
        
        self._width = width
        self._height = height
        self._format = format
        self._internal_format = internal_format
        self._data_type = data_type
        self._opts = opts or self.Options()
        
        # Generate texture using PyOpenGL
        self._texture = glGenTextures(1)
        self.init(data)
    
    def __del__(self):
        """Cleanup texture on deletion."""
        if hasattr(self, '_texture') and self._texture != 0:
            try:
                glDeleteTextures([self._texture])
            except:
                pass
    
    def init(self, data: Optional[bytes]):
        """
        Initialize the texture with data and parameters.
        
        Args:
            data: Texture data (optional)
        """
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        
        # Set texture parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, self._opts.param_min_filter)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, self._opts.param_mag_filter)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, self._opts.param_wrap_s)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, self._opts.param_wrap_t)
        
        # Upload texture data
        if data is not None:
            glTexImage2D(GL_TEXTURE_2D, 0, self._internal_format, self._width, self._height, 
                        0, self._format, self._data_type, data)
        else:
            glTexImage2D(GL_TEXTURE_2D, 0, self._internal_format, self._width, self._height, 
                        0, self._format, self._data_type, None)
        
        # Generate mipmaps if needed
        if self._opts.param_min_filter == GL_LINEAR_MIPMAP_LINEAR:
            glGenerateMipmap(GL_TEXTURE_2D)
        
        glBindTexture(GL_TEXTURE_2D, 0)
    
    def load(self, data: bytes):
        """
        Load new data into the texture.
        
        Args:
            data: Texture data
        """
        glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, self._width, self._height, 
                       self._format, self._data_type, data)
        
        # Regenerate mipmaps if needed
        if self._opts.param_min_filter == GL_LINEAR_MIPMAP_LINEAR:
            glGenerateMipmap(GL_TEXTURE_2D)
        
        glBindTexture(GL_TEXTURE_2D, 0)
    
    def bind(self, texture_unit: int = GL_TEXTURE0):
        """
        Bind the texture to a texture unit.
        
        Args:
            texture_unit: Texture unit to bind to (default: GL_TEXTURE0)
        """
        glActiveTexture(texture_unit)
        glBindTexture(GL_TEXTURE_2D, self._texture)
    
    def unbind(self, texture_unit: int = GL_TEXTURE0):
        """
        Unbind the texture from a texture unit.
        
        Args:
            texture_unit: Texture unit to unbind from (default: GL_TEXTURE0)
        """
        glActiveTexture(texture_unit)
        glBindTexture(GL_TEXTURE_2D, 0)
    
    def width(self) -> int:
        """Return the texture width in pixels."""
        return self._width
    
    def height(self) -> int:
        """Return the texture height in pixels."""
        return self._height
    
    def type(self) -> int:
        """Return the texture data type."""
        return self._data_type
    
    def format(self) -> int:
        """Return the texture format."""
        return self._format
    
    def internal_format(self) -> int:
        """Return the texture internal format."""
        return self._internal_format
    
    def is_pyramid(self) -> bool:
        """Return True if the texture uses mipmaps."""
        return self._opts.param_min_filter == GL_LINEAR_MIPMAP_LINEAR
    
    def texture_id(self) -> int:
        """Return the OpenGL texture ID."""
        return self._texture
    
    def __call__(self) -> int:
        """Return the OpenGL texture ID (allows using texture() syntax)."""
        return self.texture_id()
