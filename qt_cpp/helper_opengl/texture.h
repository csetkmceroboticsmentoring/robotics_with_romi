#pragma once

#include <type_traits>

#include <QObject>
#include <QOpenGLFunctions>

namespace opengl {

/**
 * @brief OpenGL 2D texture wrapper class that provides convenient texture management
 * 
 * This class wraps OpenGL 2D texture objects and provides methods for:
 * - Creating and managing 2D textures with various formats
 * - Setting texture parameters (filtering, wrapping, etc.)
 * - Loading texture data from memory
 * - Binding/unbinding textures to texture units
 * - Automatic cleanup of OpenGL resources
 * 
 * The class inherits from QOpenGLFunctions for OpenGL function access and QObject for memory management.
 * Supports various texture formats including RGB, RGBA, depth textures, and more.
 */
class Texture2D : private QOpenGLFunctions, protected QObject {
public:
  /**
   * @brief Structure containing texture sampling and wrapping parameters
   * 
   * This structure holds all the configurable parameters that control how the texture
   * is sampled and how texture coordinates outside the [0,1] range are handled.
   */
  struct Options {
    /**
     * @brief Default constructor with sensible default values
     * 
     * Sets up texture parameters for typical usage:
     * - Linear filtering for both minification and magnification
     * - Mirrored repeat wrapping for both S and T coordinates
     */
    Options() {};
    
    GLint param_min_filter = GL_LINEAR;        ///< Minification filter (GL_LINEAR, GL_NEAREST, GL_LINEAR_MIPMAP_LINEAR, etc.)
    GLint param_mag_filter = GL_LINEAR;        ///< Magnification filter (GL_LINEAR, GL_NEAREST)
    GLint param_wrap_s = GL_MIRRORED_REPEAT;  ///< S-coordinate wrapping mode (GL_REPEAT, GL_MIRRORED_REPEAT, GL_CLAMP_TO_EDGE, etc.)
    GLint param_wrap_t = GL_MIRRORED_REPEAT;  ///< T-coordinate wrapping mode (GL_REPEAT, GL_MIRRORED_REPEAT, GL_CLAMP_TO_EDGE, etc.)
  };
  
  /**
   * @brief Constructor that creates a 2D texture with specified parameters
   * @param ifmt Internal format (e.g., GL_RGBA8, GL_DEPTH_COMPONENT24, GL_RGB32F)
   * @param fmt Data format (e.g., GL_RGBA, GL_DEPTH_COMPONENT, GL_RGB)
   * @param type Data type (e.g., GL_UNSIGNED_BYTE, GL_FLOAT, GL_UNSIGNED_INT)
   * @param w Texture width in pixels
   * @param h Texture height in pixels
   * @param parent Parent QObject for memory management
   * @param data Pointer to initial texture data (can be nullptr for empty texture)
   * @param opts Texture sampling and wrapping options
   * 
   * @note The internal format, format, and type must be compatible with each other
   * @note If data is nullptr, the texture will be created with uninitialized content
   */
  explicit Texture2D(GLint ifmt, 
                     GLenum fmt, 
                     GLenum type, 
                     GLsizei w, 
                     GLsizei h, 
                     QObject* parent = nullptr, 
                     const void* data = 0, 
                     const Options& opts = Options());
  
  /**
   * @brief Destructor - automatically deletes the OpenGL texture object
   */
  ~Texture2D();

  /**
   * @brief Load new texture data into the existing texture
   * @param pData Pointer to the new texture data
   * 
   * Updates the texture content with new data. The data must match the texture's
   * format, type, and dimensions. This is useful for updating textures dynamically.
   */
  void load(const void* pData);

  /**
   * @brief Bind this texture to a specific texture unit
   * @param tex Texture unit to bind to (e.g., GL_TEXTURE0, GL_TEXTURE1, etc.)
   * 
   * Makes this texture the active texture for the specified texture unit.
   * All subsequent texture operations on this unit will affect this texture.
   */
  void bind(GLenum tex);
  
  /**
   * @brief Unbind the texture from the specified texture unit
   * @param tex Texture unit to unbind from
   * 
   * Removes the binding for the specified texture unit.
   * No texture will be bound to the unit after this call.
   */
  void unbind(GLenum tex);

  /**
   * @brief Get the texture width in pixels
   * @return Width of the texture
   */
  GLsizei width() const { return width_; };
  
  /**
   * @brief Get the texture height in pixels
   * @return Height of the texture
   */
  GLsizei height() const { return height_; };

  /**
   * @brief Get the OpenGL data type of the texture
   * @return OpenGL data type constant (GL_UNSIGNED_BYTE, GL_FLOAT, etc.)
   */
  GLenum type() const { return type_; };
  
  /**
   * @brief Get the OpenGL format of the texture
   * @return OpenGL format constant (GL_RGBA, GL_DEPTH_COMPONENT, etc.)
   */
  GLenum format() const { return format_; };
  
  /**
   * @brief Get the OpenGL internal format of the texture
   * @return OpenGL internal format constant (GL_RGBA8, GL_DEPTH_COMPONENT24, etc.)
   */
  GLint internalFormat() const { return internal_fmt_; };
  
  /**
   * @brief Check if the texture uses mipmapping
   * @return true if mipmapping is enabled, false otherwise
   * 
   * Determines if the texture uses mipmapping by checking if the minification
   * filter is set to a mipmap-based filter.
   */
  bool isPyramid() const { return opts_.param_min_filter == GL_LINEAR_MIPMAP_LINEAR; };

  /**
   * @brief Get the OpenGL texture object ID
   * @return OpenGL texture object ID
   */
  GLuint operator()() const { return texture_; };

private:
  // Disable default constructor, copy constructor, and assignment operator
  Texture2D() = delete;
  Texture2D(const Texture2D&) = delete;
  Texture2D& operator=(const Texture2D&) = delete;

  /**
   * @brief Initialize the texture with data and parameters
   * @param data Pointer to texture data (can be nullptr)
   * 
   * Private helper method that handles the actual OpenGL texture creation,
   * parameter setting, and data loading. Called by the constructor.
   */
  void init(const void* data);

  GLenum type_;           ///< OpenGL data type (GL_UNSIGNED_BYTE, GL_FLOAT, etc.)
  GLenum format_;         ///< OpenGL format (GL_RGBA, GL_DEPTH_COMPONENT, etc.)
  GLsizei width_;         ///< Texture width in pixels
  GLsizei height_;        ///< Texture height in pixels
  GLuint texture_;        ///< OpenGL texture object ID
  GLint internal_fmt_;    ///< OpenGL internal format (GL_RGBA8, GL_DEPTH_COMPONENT24, etc.)
  Options opts_;          ///< Texture sampling and wrapping options
};

}
