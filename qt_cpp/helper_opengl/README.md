# OpenGL Helper Library

A modern C++ wrapper library for OpenGL that provides RAII-based resource management and convenient interfaces for common OpenGL tasks. Built on top of Qt's `QOpenGLFunctions` and designed for use with Qt applications.

## Features

- **Buffer Management**: Type-safe wrappers for OpenGL buffer objects (VBOs, EBOs)
- **Shader Programs**: Easy-to-use shader compilation, linking, and uniform/attribute management
- **Texture Management**: Convenient 2D texture creation and manipulation
- **RAII**: Automatic resource cleanup using C++ destructors
- **Type Safety**: Template-based design for compile-time type checking
- **Qt Integration**: Built on Qt's OpenGL framework for cross-platform compatibility

## Dependencies

- **Qt5 OpenGL** (5.x or higher)
- **Eigen3** (3.3 or higher) - For matrix and vector operations
- **CMake** (3.8.2 or higher)
- **C++14** compatible compiler

## Building

```bash
# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build the library
make
```

### CMake Integration

To use this library in your CMake project:

```cmake
# Add the helper library directory
add_subdirectory(helper)

# Link against the library
target_link_libraries(your_target helper_opengl)
```

## Usage

### Buffer Management

The library provides type-safe buffer wrappers for managing vertex data and indices.

#### Creating Buffers

```cpp
#include "buffer.h"

using namespace opengl;

// Create a vertex buffer with initial data
std::vector<Eigen::Vector3f> vertices = {
    {0.0f, 0.5f, 0.0f},
    {-0.5f, -0.5f, 0.0f},
    {0.5f, -0.5f, 0.0f}
};
ArrayBuffer<Eigen::Vector3f> vbo(GL_STATIC_DRAW, vertices);

// Create an index buffer with maximum capacity
ElementArrayBuffer<GLuint> ebo(GL_DYNAMIC_DRAW, 100);
```

#### Buffer Operations

```cpp
// Bind buffer for use
vbo.bind();

// Load new data
std::vector<Eigen::Vector3f> newVertices = {...};
vbo.load(newVertices);

// Append data
vbo.append(moreVertices);

// Load data into specific region
vbo.load(startIndex, size, dataPtr);

// Clear buffer (resets size to 0)
vbo.clear();

// Get buffer info
size_t currentSize = vbo.size();
size_t maxCapacity = vbo.capacity();

// Unbind buffer
vbo.unbind();

// Get OpenGL buffer ID
GLuint bufferId = vbo();
```

### Shader Programs

The `Program` class simplifies shader management and provides convenient methods for setting uniforms and attributes.

#### Creating a Shader Program

```cpp
#include "program.h"

using namespace opengl;

// Vertex shader source
const char* vertexShader = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
    }
)";

// Fragment shader source
const char* fragmentShader = R"(
    #version 330 core
    out vec4 FragColor;
    uniform vec3 color;
    
    void main() {
        FragColor = vec4(color, 1.0);
    }
)";

// Create program
Program program({
    {GL_VERTEX_SHADER, vertexShader},
    {GL_FRAGMENT_SHADER, fragmentShader}
});
```

#### Using Shader Programs

```cpp
// Activate the program
program.use();

// Set uniform variables
program.setUniform("color", 1.0f, 0.0f, 0.0f);  // Red color

Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
program.setUniform("model", model);

// Set vertex attributes from buffer
program.setAttribute("aPos", vbo);

// Enable vertex attribute array
program.enableVertexAttribArray("aPos");

// ... rendering code ...

// Disable vertex attribute array
program.disableVertexAttribArray("aPos");

// Deactivate the program
program.release();
```

#### RAII Program Usage

The library provides a `Program::Use` class for automatic program state management:

```cpp
{
    // Automatically calls program.use() and enables attribute arrays
    Program::Use use(program, {"aPos", "aColor"});
    
    // Set uniforms and render
    program.setUniform("color", 1.0f, 0.0f, 0.0f);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    
} // Automatically calls program.release() and disables attribute arrays
```

### Texture Management

The `Texture2D` class provides convenient 2D texture creation and manipulation.

#### Creating Textures

```cpp
#include "texture.h"

using namespace opengl;

// Create texture with default options
int width = 512;
int height = 512;
unsigned char* imageData = loadImageData(); // Your image loading code

Texture2D texture(
    GL_RGBA8,              // Internal format
    GL_RGBA,               // Data format
    GL_UNSIGNED_BYTE,      // Data type
    width,
    height,
    nullptr,               // Parent QObject
    imageData              // Initial data
);

// Create texture with custom options
Texture2D::Options opts;
opts.param_min_filter = GL_LINEAR_MIPMAP_LINEAR;
opts.param_mag_filter = GL_LINEAR;
opts.param_wrap_s = GL_CLAMP_TO_EDGE;
opts.param_wrap_t = GL_CLAMP_TO_EDGE;

Texture2D mipmappedTexture(
    GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE,
    width, height, nullptr, imageData, opts
);
```

#### Using Textures

```cpp
// Bind texture to texture unit 0
texture.bind(GL_TEXTURE0);

// In your shader, use:
// uniform sampler2D textureSampler;
program.setUniform("textureSampler", 0);  // Texture unit 0

// ... rendering code ...

// Update texture data
texture.load(newImageData);

// Unbind texture
texture.unbind(GL_TEXTURE0);

// Get texture info
GLsizei w = texture.width();
GLsizei h = texture.height();
bool hasMipmaps = texture.isPyramid();

// Get OpenGL texture ID
GLuint texId = texture();
```

## Complete Example

Here's a complete example that combines all components:

```cpp
#include "buffer.h"
#include "program.h"
#include "texture.h"

#include <QOpenGLWidget>

class MyGLWidget : public QOpenGLWidget {
protected:
    void initializeGL() override {
        // Vertex data
        std::vector<Eigen::Vector3f> vertices = {
            {-0.5f, -0.5f, 0.0f},
            { 0.5f, -0.5f, 0.0f},
            { 0.0f,  0.5f, 0.0f}
        };
        
        // Create vertex buffer
        vbo = new opengl::ArrayBuffer<Eigen::Vector3f>(
            GL_STATIC_DRAW, vertices, this
        );
        
        // Shader sources
        const char* vs = R"(
            #version 330 core
            layout (location = 0) in vec3 aPos;
            void main() { gl_Position = vec4(aPos, 1.0); }
        )";
        
        const char* fs = R"(
            #version 330 core
            out vec4 FragColor;
            uniform vec3 color;
            void main() { FragColor = vec4(color, 1.0); }
        )";
        
        // Create shader program
        program = new opengl::Program({
            {GL_VERTEX_SHADER, vs},
            {GL_FRAGMENT_SHADER, fs}
        }, this);
    }
    
    void paintGL() override {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Use RAII for automatic cleanup
        opengl::Program::Use use(*program, {"aPos"});
        
        program->setUniform("color", 1.0f, 0.0f, 0.0f);
        program->setAttribute("aPos", *vbo);
        
        glDrawArrays(GL_TRIANGLES, 0, 3);
    }

private:
    opengl::ArrayBuffer<Eigen::Vector3f>* vbo;
    opengl::Program* program;
};
```

## API Reference

### Buffer<T, target>

Template class for OpenGL buffer objects.

**Template Parameters:**
- `T`: Data type stored in the buffer
- `target`: OpenGL buffer target (GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER, etc.)

**Type Aliases:**
- `ArrayBuffer<T>`: Buffer with target GL_ARRAY_BUFFER
- `ElementArrayBuffer<T>`: Buffer with target GL_ELEMENT_ARRAY_BUFFER

**Methods:**
- `Buffer(usage, max_items, parent)`: Create buffer with max capacity
- `Buffer(usage, data, parent)`: Create buffer with initial data
- `size()`: Get current number of items
- `capacity()`: Get maximum capacity
- `bind()`: Bind buffer to context
- `unbind()`: Unbind buffer
- `clear()`: Reset size to 0
- `load(data)`: Replace buffer contents
- `append(data)`: Append to buffer
- `load(start, size, data)`: Update specific region
- `operator()()`: Get OpenGL buffer ID

### Program

Class for OpenGL shader programs.

**Methods:**
- `Program(shaders, parent)`: Create and link shader program
- `use()`: Activate program
- `release()`: Deactivate program
- `enableVertexAttribArray(name)`: Enable attribute array
- `disableVertexAttribArray(name)`: Disable attribute array
- `setUniform(name, ...)`: Set uniform variable (multiple overloads)
- `setAttribute(name, ...)`: Set vertex attribute (multiple overloads)

**Inner Class:**
- `Program::Use`: RAII wrapper for automatic program state management

### Texture2D

Class for 2D textures.

**Nested Types:**
- `Texture2D::Options`: Structure for texture parameters

**Methods:**
- `Texture2D(ifmt, fmt, type, w, h, parent, data, opts)`: Create texture
- `load(data)`: Update texture data
- `bind(unit)`: Bind to texture unit
- `unbind(unit)`: Unbind from texture unit
- `width()`: Get texture width
- `height()`: Get texture height
- `type()`: Get data type
- `format()`: Get format
- `internalFormat()`: Get internal format
- `isPyramid()`: Check if mipmaps are enabled
- `operator()()`: Get OpenGL texture ID

## Design Philosophy

This library follows several key design principles:

1. **RAII (Resource Acquisition Is Initialization)**: All OpenGL resources are automatically cleaned up when objects are destroyed, preventing resource leaks.

2. **Type Safety**: Template-based design ensures type mismatches are caught at compile time.

3. **Qt Integration**: Built on Qt's OpenGL framework, the library inherits Qt's cross-platform capabilities and integrates seamlessly with Qt applications.

4. **Convenience Without Sacrifice**: The library provides convenient high-level interfaces while still allowing access to underlying OpenGL objects when needed.

5. **Explicit is Better**: Operations are explicit and predictable, making code easier to understand and debug.

## Contributing

Contributions are welcome! Please ensure all code:
- Follows the existing code style
- Includes appropriate documentation
- Compiles without warnings
- Maintains the RAII design pattern

## See Also

- Python version: See `../helper_py/` for a Python/PyQt6 equivalent of this library
- Qt OpenGL Documentation: https://doc.qt.io/qt-5/qtopengl-index.html
- Eigen Documentation: https://eigen.tuxfamily.org/

