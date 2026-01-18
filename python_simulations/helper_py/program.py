"""
Shader program management for OpenGL using PyOpenGL.
Uses PySide6's QObject for lifecycle management only.
"""

from typing import List, Tuple, Optional, Union
import numpy as np
from PySide6.QtCore import QObject
from OpenGL.GL import *

from .buffer import Buffer


class Program(QObject):
    """
    OpenGL shader program wrapper class using PyOpenGL.
    
    This class manages shader compilation, linking, and provides methods
    for setting uniforms and vertex attributes.
    """
    
    class Use:
        """
        Context manager for using a shader program.
        
        Automatically enables and disables vertex attribute arrays.
        """
        
        def __init__(self, program: 'Program', attribute_names: Optional[List[str]] = None):
            """
            Initialize the context manager.
            
            Args:
                program: The Program to use
                attribute_names: List of attribute names to enable
            """
            self._program = program
            self._attribute_names = attribute_names or []
            program.use()
            
            for name in self._attribute_names:
                program.enable_vertex_attrib_array(name)
        
        def __enter__(self):
            return self
        
        def __exit__(self, exc_type, exc_val, exc_tb):
            for name in self._attribute_names:
                self._program.disable_vertex_attrib_array(name)
            self._program.release()
    
    def __init__(self, shaders: List[Tuple[int, str]], parent: Optional[QObject] = None):
        """
        Initialize a new shader program.
        
        Args:
            shaders: List of (shader_type, source_code) tuples
            parent: Parent QObject for memory management
        
        Raises:
            RuntimeError: If shader compilation or linking fails
        """
        super().__init__(parent)
        
        # Create program using PyOpenGL
        self._program = glCreateProgram()
        
        # Compile and attach shaders
        shader_ids = []
        for shader_type, source_code in shaders:
            shader = glCreateShader(shader_type)
            glShaderSource(shader, source_code)
            self._compile(shader)
            glAttachShader(self._program, shader)
            shader_ids.append(shader)
        
        # Link program
        glLinkProgram(self._program)
        
        # Check link status
        link_status = glGetProgramiv(self._program, GL_LINK_STATUS)
        if link_status != GL_TRUE:
            error_log = glGetProgramInfoLog(self._program)
            error_msg = "Shader program linking failed"
            if error_log:
                error_msg = f"Shader program linking failed:\n{error_log.decode() if isinstance(error_log, bytes) else error_log}"
            raise RuntimeError(error_msg)
        
        # Clean up shaders (they're now part of the program)
        for shader in shader_ids:
            glDeleteShader(shader)
    
    def _compile(self, shader: int):
        """
        Compile a shader and check for errors.
        
        Args:
            shader: Shader ID
        
        Raises:
            RuntimeError: If compilation fails
        """
        glCompileShader(shader)
        status = glGetShaderiv(shader, GL_COMPILE_STATUS)
        
        if status != GL_TRUE:
            error_log = glGetShaderInfoLog(shader)
            error_msg = "Shader compilation failed"
            if error_log:
                error_msg = f"Shader compilation failed:\n{error_log.decode() if isinstance(error_log, bytes) else error_log}"
            raise RuntimeError(error_msg)
    
    def __del__(self):
        """Cleanup program on deletion."""
        if hasattr(self, '_program') and self._program != 0:
            try:
                glDeleteProgram(self._program)
            except:
                pass
    
    def use(self):
        """Activate this shader program."""
        glUseProgram(self._program)
    
    def release(self):
        """Deactivate the current shader program."""
        glUseProgram(0)
    
    def enable_vertex_attrib_array(self, name: str):
        """
        Enable a vertex attribute array.
        
        Args:
            name: Name of the vertex attribute in the shader
        """
        loc = glGetAttribLocation(self._program, name)
        if loc >= 0:
            glEnableVertexAttribArray(loc)
    
    def disable_vertex_attrib_array(self, name: str):
        """
        Disable a vertex attribute array.
        
        Args:
            name: Name of the vertex attribute in the shader
        """
        loc = glGetAttribLocation(self._program, name)
        if loc >= 0:
            glDisableVertexAttribArray(loc)
    
    def get_attribute_location(self, name: str) -> int:
        """
        Get the location of a vertex attribute.
        
        Args:
            name: Name of the vertex attribute in the shader
            
        Returns:
            Attribute location, or -1 if not found
        """
        return glGetAttribLocation(self._program, name)
    
    def set_uniform(self, name: str, *args):
        """
        Set a uniform variable.
        
        Args:
            name: Name of the uniform variable in the shader
            *args: Uniform value(s) - can be:
                - Single float/int
                - 2, 3, or 4 floats for vectors
                - numpy array for matrices (3x3 or 4x4)
        """
        loc = glGetUniformLocation(self._program, name)
        if loc < 0:
            return  # Uniform not found, silently ignore
        
        if len(args) == 1:
            value = args[0]
            if isinstance(value, int):
                # Use integer uniform (important for samplers like sampler2D)
                glUniform1i(loc, value)
            elif isinstance(value, float):
                glUniform1f(loc, value)
            elif isinstance(value, np.ndarray):
                if value.shape == (3, 3):
                    # GL_TRUE tells OpenGL to transpose from row-major (numpy) to column-major (OpenGL)
                    glUniformMatrix3fv(loc, 1, GL_TRUE, value)
                elif value.shape == (4, 4):
                    # GL_TRUE tells OpenGL to transpose from row-major (numpy) to column-major (OpenGL)
                    glUniformMatrix4fv(loc, 1, GL_TRUE, value)
                else:
                    raise ValueError(f"Matrix must be 3x3 or 4x4, got {value.shape}")
            else:
                raise TypeError(f"Unsupported uniform type: {type(value)}")
        elif len(args) == 2:
            glUniform2f(loc, float(args[0]), float(args[1]))
        elif len(args) == 3:
            glUniform3f(loc, float(args[0]), float(args[1]), float(args[2]))
        elif len(args) == 4:
            glUniform4f(loc, float(args[0]), float(args[1]), 
                       float(args[2]), float(args[3]))
        else:
            raise ValueError(f"Unsupported number of arguments: {len(args)}")
    
    def set_uniform_matrix(self, name: str, matrix: np.ndarray, transpose: bool = True):
        """
        Set a uniform variable to a matrix.
        
        Args:
            name: Name of the uniform variable in the shader
            matrix: 3x3 or 4x4 numpy array (matrix)
            transpose: Whether to transpose the matrix (default True for numpy row-major to OpenGL column-major)
        """
        loc = glGetUniformLocation(self._program, name)
        if loc < 0:
            return  # Uniform not found, silently ignore
        
        transpose_gl = GL_TRUE if transpose else GL_FALSE
        
        if matrix.shape == (3, 3):
            glUniformMatrix3fv(loc, 1, transpose_gl, matrix)
        elif matrix.shape == (4, 4):
            glUniformMatrix4fv(loc, 1, transpose_gl, matrix)
        else:
            raise ValueError(f"Matrix must be 3x3 or 4x4, got {matrix.shape}")
    
    def set_attribute(self, name: str, data: Union[List, Buffer, np.ndarray], 
                     components: Optional[int] = None):
        """
        Set a vertex attribute from data.
        
        Args:
            name: Name of the vertex attribute in the shader
            data: Attribute data - can be:
                - List of vectors (e.g., list of [x, y] or [x, y, z])
                - Buffer object
                - numpy array
            components: Number of components per attribute (auto-detected if None)
        
        This method sets up vertex attribute pointers for rendering.
        """
        loc = glGetAttribLocation(self._program, name)
        if loc < 0:
            return  # Attribute not found, silently ignore
        
        if isinstance(data, Buffer):
            # Buffer-based attribute
            data.bind()
            if components is None:
                components = 3  # Default to 3 components
            
            glVertexAttribPointer(loc, components, GL_FLOAT, GL_FALSE, 0, None)
            # Don't unbind here - keep the buffer bound for the draw call
        elif isinstance(data, (list, np.ndarray)):
            # Direct data attribute
            if isinstance(data, list):
                np_data = np.array(data, dtype=np.float32)
            else:
                np_data = data.astype(np.float32)
            
            if components is None:
                if len(np_data.shape) == 1:
                    components = 1
                else:
                    components = np_data.shape[1]
            
            # Note: For direct data, we'd need to create a temporary buffer
            # This is a simplified version - in practice, you should use buffers
            raise NotImplementedError("Direct data attributes require using Buffer objects")
        else:
            raise TypeError(f"Unsupported attribute data type: {type(data)}")
    
    def program_id(self) -> int:
        """Return the OpenGL program ID."""
        return self._program
