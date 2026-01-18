#pragma once

#include <list>
#include <type_traits>

#include <QObject>
#include <QOpenGLFunctions>

#include <Eigen/Dense>

#include "buffer.h"

namespace opengl {

/**
 * @brief OpenGL shader program wrapper class that manages shader compilation and linking
 * 
 * This class provides a convenient interface for:
 * - Creating and managing OpenGL shader programs
 * - Compiling and linking vertex, fragment, geometry shaders etc
 * - Setting uniform variables and vertex attributes
 * - Managing program state (use/release)
 * - Automatic cleanup of OpenGL resources
 * 
 * The class inherits from QOpenGLFunctions for OpenGL function access and QObject for memory management.
 */
class Program : private QOpenGLFunctions, protected QObject {
public:
  /**
   * @brief Constructor that creates and links a shader program from source code
   * @param srcs List of shader source pairs (shader type, source code)
   * @param parent Parent QObject for memory management
   * 
   * @note The shader types should be GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, GL_GEOMETRY_SHADER, etc.
   * @note The source code strings should be null-terminated C-style strings.
   */
  Program(const std::list<std::pair<GLenum, const GLchar*>>& srcs, QObject *parent = nullptr);
  
  /**
   * @brief Destructor - automatically deletes the OpenGL shader program
   */
  ~Program();

  /**
   * @brief Activate this shader program for use in rendering
   * 
   * Makes this program the current active shader program.
   * All subsequent rendering calls will use this program's shaders.
   */
  void use();
  
  /**
   * @brief Deactivate this shader program
   * 
   * Removes this program from the current OpenGL state.
   * No shader program will be active after this call.
   */
  void release();

  /**
   * @brief Enable a vertex attribute array by name
   * @param name Name of the vertex attribute in the shader
   * 
   * Enables the vertex attribute array so it can be used in rendering.
   */
  void enableVertexAttribArray(const char* name);
  
  /**
   * @brief Disable a vertex attribute array by name
   * @param name Name of the vertex attribute in the shader
   * 
   * Disables the vertex attribute array to prevent it from being used in rendering.
   */
  void disableVertexAttribArray(const char* name);

  /**
   * @brief Set a uniform variable to a single float value
   * @param name Name of the uniform variable in the shader
   * @param value Float value to set
   */
  void setUniform(const char* name, const float value);
  
  /**
   * @brief Set a uniform variable to a 2D vector (x, y)
   * @param name Name of the uniform variable in the shader
   * @param x X component value
   * @param y Y component value
   */
  void setUniform(const char* name, const float x, const float y);
  
  /**
   * @brief Set a uniform variable to a 3D vector (x, y, z)
   * @param name Name of the uniform variable in the shader
   * @param x X component value
   * @param y Y component value
   * @param z Z component value
   */
  void setUniform(const char* name, const float x, const float y, const float z);
  
  /**
   * @brief Set a uniform variable to a 4D vector (x, y, z, w)
   * @param name Name of the uniform variable in the shader
   * @param x X component value
   * @param y Y component value
   * @param z Z component value
   * @param w W component value
   */
  void setUniform(const char* name, const float& x, const float& y, const float& z, const float& w);
  
  /**
   * @brief Set a uniform variable to a 3x3 matrix
   * @param name Name of the uniform variable in the shader
   * @param matrix 3x3 Eigen matrix to set
   * @param transpose Whether to transpose the matrix before sending to GPU
   */
  void setUniform(const char* name, const Eigen::Matrix3f& matrix, const bool transpose = false);
  
  /**
   * @brief Set a uniform variable to a 4x4 matrix
   * @param name Name of the uniform variable in the shader
   * @param matrix 4x4 Eigen matrix to set
   * @param transpose Whether to transpose the matrix before sending to GPU
   */
  void setUniform(const char* name, const Eigen::Matrix4f& matrix, const bool transpose = false);

  /**
   * @brief Set a vertex attribute from a vector of Eigen vectors
   * @tparam T Data type (float, double, etc.)
   * @tparam N Vector dimension (2, 3, 4, etc.)
   * @param name Name of the vertex attribute in the shader
   * @param attrib_data Vector of N-dimensional vectors
   * 
   * This method sets up vertex attribute pointers for immediate mode rendering.
   */
  template <typename T, int N>
  void setAttribute(const char* name, const std::vector<Eigen::Matrix<T, N, 1>>& attrib_data);

  /**
   * @brief Set a vertex attribute from a buffer containing Eigen vectors
   * @tparam T Data type (float, double, etc.)
   * @tparam N Vector dimension (2, 3, 4, etc.)
   * @tparam target OpenGL buffer target (GL_ARRAY_BUFFER, etc.)
   * @param name Name of the vertex attribute in the shader
   * @param buffer Buffer containing the attribute data
   * 
   * This method sets up vertex attribute pointers for buffer-based rendering.
   */
  template <typename T, int N, GLenum target>
  void setAttribute(const char* name, const Buffer<Eigen::Matrix<T, N, 1>, target>& buffer);

  /**
   * @brief Set a vertex attribute from a buffer containing scalar values
   * @tparam T Data type (float, double, int, etc.)
   * @tparam target OpenGL buffer target (GL_ARRAY_BUFFER, etc.)
   * @param name Name of the vertex attribute in the shader
   * @param buffer Buffer containing the attribute data
   * 
   * This method sets up vertex attribute pointers for scalar attribute data.
   */
  template <typename T, GLenum target>
  void setAttribute(const char* name, const Buffer<T, target>& buffer);

  /**
   * @brief RAII wrapper class for automatic program state management
   * 
   * This class automatically calls use() on construction and release() on destruction,
   * ensuring proper cleanup even when exceptions occur. Also automatically enables and 
   * disables the specified vertex attribute arrays.
   * 
   * Usage example:
   * @code
   * {
   *   Program::Use use(program, {"vertices", "colors"});  // Automatically calls program.use()
   *   // ... rendering code ...
   * } // Automatically calls program.release() when scope ends
   */
  class Use {
  public:
    /**
     * @brief Constructor - automatically activates the program
     * @param program Reference to the program to activate
     * @param str_list Optional list of strings (currently unused)
     */
    Use(Program& program, const std::initializer_list<std::string>& str_list = {});
    
    /**
     * @brief Destructor - automatically deactivates the program
     */
    ~Use();
   private:
     Program& program_;                ///< Reference to the program to manage
     const std::list<std::string> str_list_;  ///< List of vertex attribute names to enable/disable
  };

private:
  // Disable default constructor, copy constructor, and assignment operator
  Program() = delete;
  Program(const Program&) = delete;
  Program& operator=(const Program&) = delete;

  /**
   * @brief Compile a shader and check for compilation errors
   * @param shader OpenGL shader object ID to compile
   * 
   * Compiles the shader and throws an exception if compilation fails.
   */
  void compile(const GLuint shader);

  /**
   * @brief Get the OpenGL data type corresponding to template type T
   * @tparam T C++ data type
   * @return OpenGL data type constant (GL_FLOAT, GL_INT, etc.)
   * 
   * This template method maps C++ types to OpenGL types for vertex attribute setup.
   */
  template <typename T>
  GLenum type() const {
    if (std::is_same<T, int>::value) return GL_INT;
    if (std::is_same<T, float>::value) return GL_FLOAT;
    return GL_FLOAT; // Default to float for unknown types
  }

  GLuint program; ///< OpenGL shader program object ID
};

template <typename T, int N>
void Program::setAttribute(const char* name, const std::vector<Eigen::Matrix<T, N, 1>>& attrib_data) {
  const GLuint pos = glGetAttribLocation(program, name); // Get attribute location by name
  glVertexAttribPointer(pos, N, type<T>(), GL_FALSE, sizeof(Eigen::Matrix<T, N, 1>), attrib_data.data()); // Set attribute pointer
}

template <typename T, int N, GLenum target>
void Program::setAttribute(const char* name, const Buffer<Eigen::Matrix<T, N, 1>, target>& buffer) {
  const GLuint pos = glGetAttribLocation(program, name); // Get attribute location by name
  glBindBuffer(target, buffer()); // Bind the buffer to the target
  glVertexAttribPointer(pos, N, type<T>(), GL_FALSE, sizeof(Eigen::Matrix<T, N, 1>), 0); // Set attribute pointer with offset 0
  glBindBuffer(target, 0); // Unbind the buffer
}

template <typename T, GLenum target>
void Program::setAttribute(const char* name, const Buffer<T, target>& buffer) {
  const GLuint pos = glGetAttribLocation(program, name); // Get attribute location by name
  glBindBuffer(target, buffer()); // Bind the buffer to the target
  glVertexAttribPointer(pos, 1, type<T>(), GL_FALSE, sizeof(T), 0); // Set attribute pointer for scalar data
  glBindBuffer(target, 0); // Unbind the buffer
}

}
