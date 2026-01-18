#pragma once

#include <vector>
#include <type_traits>

#include <QObject>
#include <QOpenGLFunctions>

namespace opengl {

/**
 * @brief Forward declaration of the Buffer template class
 * @tparam T Data type to be stored in the buffer
 * @tparam target OpenGL buffer target (e.g., GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER etc)
 */
template <typename T, GLenum target>
class Buffer;

/**
 * @brief Type alias for array buffer - stores vertex attributes
 * @tparam T Data type for vertex attributes
 */
template <typename T>
using ArrayBuffer = Buffer<T, GL_ARRAY_BUFFER>;

/**
 * @brief Type alias for element array buffer - stores indices for indexed drawing
 * @tparam T Data type for indices (typically GLuint)
 */
template <typename T>
using ElementArrayBuffer = Buffer<T, GL_ELEMENT_ARRAY_BUFFER>;

/**
 * @brief OpenGL buffer wrapper class that provides RAII and convenient buffer management
 * 
 * This class wraps OpenGL buffer objects and provides methods for:
 * - Creating and destroying buffers
 * - Binding/unbinding buffers
 * - Loading and updating buffer data
 * - Managing buffer capacity and size
 * 
 * @tparam T Data type to be stored in the buffer
 * @tparam target OpenGL buffer target (GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER, etc.)
 */
template <typename T, GLenum target>
class Buffer : private QOpenGLFunctions, protected QObject {
public:
  /**
   * @brief Constructor for creating a buffer with specified usage and maximum capacity
   * @param usage OpenGL usage hint (e.g., GL_STATIC_DRAW, GL_DYNAMIC_DRAW)
   * @param max_items Maximum number of items the buffer can hold
   * @param parent Parent QObject for memory management
   */
  Buffer(const GLenum usage, const size_t max_items, QObject* parent = nullptr);
  
  /**
   * @brief Constructor for creating a buffer with initial data
   * @param usage OpenGL usage hint
   * @param data Initial data to populate the buffer
   * @param parent Parent QObject for memory management
   */
  Buffer(const GLenum usage, const std::vector<T>& data, QObject* parent = nullptr);
  
  /**
   * @brief Destructor - automatically deletes the OpenGL buffer
   */
  ~Buffer();

  /**
   * @brief Get the current number of items in the buffer
   * @return Number of items currently stored
   */
  size_t size() const;
  
  /**
   * @brief Get the maximum capacity of the buffer
   * @return Maximum number of items the buffer can hold
   */
  size_t capacity() const { return capacity_; };

  /**
   * @brief Bind this buffer to the OpenGL context
   */
  void bind();
  
  /**
   * @brief Unbind the buffer from the OpenGL context
   */
  void unbind();

  /**
   * @brief Clear the buffer contents (sets size to 0, data remains in GPU memory)
   */
  void clear();

  /**
   * @brief Load new data into the buffer, replacing existing content
   * @param data Vector of data to load
   */
  void load(const std::vector<T>& data);
  
  /**
   * @brief Append data to the end of existing buffer content
   * @param data Vector of data to append
   */
  void append(const std::vector<T>& data);

  /**
   * @brief Load data into a specific region of the buffer
   * @param start Starting index in the buffer
   * @param size Number of items to load
   * @param data Pointer to the data to load
   */
  void load(const size_t start, const size_t size, const T* data);

  /**
   * @brief Get the OpenGL buffer ID
   * @return OpenGL buffer object ID
   */
  GLuint operator()() const { return buffer_; }

private:
  // Disable default constructor and copy constructor
  Buffer() = delete;
  Buffer(const Buffer&) = delete;

  size_t size_;           ///< Current number of items in the buffer
  mutable GLuint buffer_; ///< OpenGL buffer object ID
  const size_t capacity_; ///< Maximum capacity of the buffer
};

template <typename T, GLenum target>
Buffer<T, target>::Buffer(const GLenum usage, const std::vector<T>& data, QObject* parent)
  : QObject(parent),
    size_(data.size()),
    capacity_(data.size()) {
  initializeOpenGLFunctions();  // Initialize OpenGL function pointers
  glGenBuffers(1, &buffer_);   // Generate a new OpenGL buffer object
  glBindBuffer(target, buffer_); // Bind the buffer to the specified target
  glBufferData(target, sizeof(T) * data.size(), data.data(), usage); // Allocate and populate buffer
  glBindBuffer(target, 0);      // Unbind the buffer
}

template <typename T, GLenum target>
Buffer<T, target>::Buffer(const GLenum usage, const size_t max_items, QObject* parent)
  : QObject(parent),
    size_(0),
    capacity_(max_items) {
  initializeOpenGLFunctions();  // Initialize OpenGL function pointers
  glGenBuffers(1, &buffer_);   // Generate a new OpenGL buffer object
  glBindBuffer(target, buffer_); // Bind the buffer to the specified target
  glBufferData(target, sizeof(T) * max_items, nullptr, usage); // Allocate buffer with no data
  glBindBuffer(target, 0);      // Unbind the buffer
}

template <typename T, GLenum target>
Buffer<T, target>::~Buffer() {
  glDeleteBuffers(1, &buffer_);
}

template <typename T, GLenum target>
size_t Buffer<T, target>::size() const { return size_; }

template <typename T, GLenum target>
void Buffer<T, target>::bind() {
  glBindBuffer(target, buffer_);
}

template <typename T, GLenum target>
void Buffer<T, target>::unbind() {
  glBindBuffer(target, 0);
}

template <typename T, GLenum target>
void Buffer<T, target>::clear() {
  size_ = 0;
}

template <typename T, GLenum target>
void Buffer<T, target>::load(const std::vector<T>& data) {
  glBindBuffer(target, buffer_);
  glBufferSubData(target, 0, data.size() * sizeof(T), data.data()); // Update buffer data
  glBindBuffer(target, 0);
  size_ = data.size(); // Update the size to reflect new data
}

template <typename T, GLenum target>
void Buffer<T, target>::append(const std::vector<T>& data) {
  glBindBuffer(target, buffer_);
  glBufferSubData(target, size_ * sizeof(T), data.size() * sizeof(T), data.data()); // Append data
  glBindBuffer(target, 0);
  size_ += data.size(); // Increase size by the amount of appended data
}

template <typename T, GLenum target>
void Buffer<T, target>::load(const size_t start, const size_t size, const T* data) {
  glBindBuffer(target, buffer_);
  glBufferSubData(target, start * sizeof(T), size * sizeof(T), data); // Update specific region
  glBindBuffer(target, 0);
}

}
