#include "texture.h"

namespace opengl {

Texture2D::Texture2D(GLint ifmt, GLenum fmt, GLenum type, GLsizei w, GLsizei h, QObject* parent, const void* data, const Options& opts)
    : QObject(parent),
      type_(type),           // Store the data type for later use
      format_(fmt),          // Store the data format for later use
      width_(w),             // Store the texture width
      height_(h),            // Store the texture height
      texture_(0),           // Initialize texture ID to 0
      internal_fmt_(ifmt),   // Store the internal format for GPU storage
      opts_(opts) {          // Store the texture options
  initializeOpenGLFunctions(); // Initialize OpenGL function pointers for Qt
  glGenTextures(1, &texture_); // Generate a new OpenGL texture object
  init(data);                   // Initialize the texture with data and parameters
}

Texture2D::~Texture2D() {
  glDeleteTextures(1, &texture_); // Delete the OpenGL texture object
}

void Texture2D::init(const void* data) {
  glActiveTexture(GL_TEXTURE0); // Activate texture unit 0 (the default unit)
  glBindTexture(GL_TEXTURE_2D, texture_); // Bind our texture to the 2D texture target
  
  // Set texture filtering parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, opts_.param_min_filter); // Minification filter
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, opts_.param_mag_filter); // Magnification filter
  
  // Set texture wrapping parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, opts_.param_wrap_s); // S-coordinate wrapping
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, opts_.param_wrap_t); // T-coordinate wrapping
  
  // Allocate texture memory and optionally load data
  // Parameters: target, level, internalFormat, width, height, border, format, type, data
  glTexImage2D(GL_TEXTURE_2D, 0, internal_fmt_, width_, height_, 0, format_, type_, data);
  
  // Generate mipmaps if trilinear filtering is enabled
  if (opts_.param_min_filter == GL_LINEAR_MIPMAP_LINEAR) {
    glGenerateMipmap(GL_TEXTURE_2D); // Generate complete mipmap chain
  }
  
  glBindTexture(GL_TEXTURE_2D, 0); // Unbind the texture to avoid affecting other operations
}

void Texture2D::bind(GLenum tex) {
  glActiveTexture(tex); // Activate the specified texture unit
  glBindTexture(GL_TEXTURE_2D, texture_); // Bind our texture to the 2D texture target
}

void Texture2D::unbind(GLenum tex) {
  glActiveTexture(tex); // Activate the specified texture unit
  glBindTexture(GL_TEXTURE_2D, 0); // Bind texture ID 0 (no texture) to unbind
}

void Texture2D::load(const void* data) {
  glActiveTexture(GL_TEXTURE0); // Activate texture unit 0
  glBindTexture(GL_TEXTURE_2D, texture_); // Bind our texture
  
  // Update texture data without reallocating memory
  // Parameters: target, level, xoffset, yoffset, width, height, format, type, data
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, format_, type_, data);
  
  // Regenerate mipmaps if mipmapping is enabled (needed after data update)
  if (opts_.param_min_filter == GL_LINEAR_MIPMAP_LINEAR) {
    glGenerateMipmap(GL_TEXTURE_2D); // Regenerate complete mipmap chain
  }
  
  glBindTexture(GL_TEXTURE_2D, 0); // Unbind the texture
}

}
