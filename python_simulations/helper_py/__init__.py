"""
OpenGL helper library for Python using PyOpenGL.

This package provides Python wrappers for OpenGL functionality similar to the C++ helper library.
It includes classes for managing OpenGL buffers, shader programs, and textures.

Example usage:
    from helper_py import Buffer, Program, Texture2D
    from helper_py.buffer import ArrayBuffer, ElementArrayBuffer
    
    # Create a vertex buffer
    vertices = ArrayBuffer(GL_STATIC_DRAW, data=[[0, 0], [1, 0], [0, 1]])
    
    # Create a shader program
    vertex_shader = '''
        #version 330 core
        layout (location = 0) in vec2 aPos;
        void main() {
            gl_Position = vec4(aPos, 0.0, 1.0);
        }
    '''
    fragment_shader = '''
        #version 330 core
        out vec4 FragColor;
        void main() {
            FragColor = vec4(1.0, 0.0, 0.0, 1.0);
        }
    '''
    program = Program([
        (GL_VERTEX_SHADER, vertex_shader),
        (GL_FRAGMENT_SHADER, fragment_shader)
    ])
    
    # Use the program
    with Program.Use(program, ["aPos"]):
        # ... rendering code ...
        pass
"""

from .buffer import Buffer, ArrayBuffer, ElementArrayBuffer
from .program import Program
from .texture import Texture2D

# Re-export commonly used OpenGL constants from PyOpenGL for convenience
from OpenGL.GL import (
    # Buffer constants
    GL_ARRAY_BUFFER,
    GL_ELEMENT_ARRAY_BUFFER,
    GL_STATIC_DRAW,
    GL_DYNAMIC_DRAW,
    GL_STREAM_DRAW,
    
    # Shader constants
    GL_VERTEX_SHADER,
    GL_FRAGMENT_SHADER,
    GL_GEOMETRY_SHADER,
    GL_COMPUTE_SHADER,
    GL_FLOAT,
    
    # Texture constants
    GL_TEXTURE0,
    GL_TEXTURE1,
    GL_TEXTURE2,
    GL_TEXTURE3,
    GL_TEXTURE4,
    GL_TEXTURE5,
    GL_TEXTURE6,
    GL_TEXTURE7,
    GL_TEXTURE_2D,
    GL_RGB,
    GL_RGBA,
    GL_UNSIGNED_BYTE,
    GL_LINEAR,
    GL_NEAREST,
    GL_LINEAR_MIPMAP_LINEAR,
    GL_REPEAT,
    GL_MIRRORED_REPEAT,
    GL_CLAMP_TO_EDGE,
)

__all__ = [
    # Classes
    'Buffer',
    'ArrayBuffer',
    'ElementArrayBuffer',
    'Program',
    'Texture2D',
    
    # OpenGL constants - Buffer
    'GL_ARRAY_BUFFER',
    'GL_ELEMENT_ARRAY_BUFFER',
    'GL_STATIC_DRAW',
    'GL_DYNAMIC_DRAW',
    'GL_STREAM_DRAW',
    
    # OpenGL constants - Program
    'GL_VERTEX_SHADER',
    'GL_FRAGMENT_SHADER',
    'GL_GEOMETRY_SHADER',
    'GL_COMPUTE_SHADER',
    
    # OpenGL constants - Texture
    'GL_TEXTURE0',
    'GL_TEXTURE1',
    'GL_TEXTURE2',
    'GL_TEXTURE3',
    'GL_TEXTURE4',
    'GL_TEXTURE5',
    'GL_TEXTURE6',
    'GL_TEXTURE7',
    'GL_LINEAR',
    'GL_NEAREST',
    'GL_LINEAR_MIPMAP_LINEAR',
    'GL_REPEAT',
    'GL_MIRRORED_REPEAT',
    'GL_CLAMP_TO_EDGE',
]

__version__ = '1.0.0'

