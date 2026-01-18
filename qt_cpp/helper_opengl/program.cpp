#include "program.h"
#include <QDebug>

namespace opengl {

Program::Use::Use(Program& program, const std::initializer_list<std::string>& str_list)
  : program_(program),
    str_list_(std::move(str_list)) {
  program_.use(); // Activate the shader program
  
  // Automatically enable all specified vertex attribute arrays
  for (const std::string& name : str_list_) {
    program_.enableVertexAttribArray(name.c_str());
  }
}

Program::Use::~Use() {
  // Disable all vertex attribute arrays that were enabled in the constructor
  for (const std::string& name : str_list_) {
    program_.disableVertexAttribArray(name.c_str());
  }
  program_.release(); // Deactivate the shader program
}

Program::Program(const std::list<std::pair<GLenum, const GLchar*>>& shaders, QObject *parent)
    : QObject(parent),
      program(0) {
  initializeOpenGLFunctions(); // Initialize OpenGL function pointers for Qt
  
  // Create a new OpenGL program object
  program = glCreateProgram();
  
  // Process each shader source in the list
  for (const auto& item : shaders) {
    GLuint shader = glCreateShader(item.first); // Create shader of specified type
    const GLchar* ptr = item.second;            // Get pointer to shader source code
    glShaderSource(shader, 1, &ptr, 0);        // Set the shader source code
    compile(shader);                            // Compile the shader and check for errors
    glAttachShader(program, shader);            // Attach compiled shader to program
    glDeleteShader(shader);                     // Delete shader object (no longer needed)
  }
  
  // Link all attached shaders into a single executable program
  glLinkProgram(program);
}

Program::~Program() {
  glDeleteProgram(program);
}

void Program::use() {
  glUseProgram(program);
}

void Program::release() {
  glUseProgram(0);
}

void Program::compile(const GLuint shader) {
  glCompileShader(shader); // Compile the shader
  
  // Check if compilation was successful
  GLint status = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
  
  if (status != GL_TRUE) {
    // Compilation failed - get the error log
    GLint length;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    
    // Create a string to hold the error message
    std::string str_err(length, 0);
    glGetShaderInfoLog(shader, length, nullptr, &str_err[0]);
    
    // Display the error and terminate (in production, consider throwing an exception)
    qDebug() << "Shader Error:\n\n" << str_err.c_str() << "\n\n";
    exit(0);
  }
}

void Program::setUniform(const char* name, const float value) {
  const GLuint loc = glGetUniformLocation(program, name); // Get uniform location by name
  glUniform1f(loc, value); // Set uniform to single float value
}

void Program::setUniform(const char* name, const float x, const float y) {
  const GLint loc = glGetUniformLocation(program, name); // Get uniform location by name
  glUniform2f(loc, x, y); // Set uniform to 2D vector (x, y)
}

void Program::setUniform(const char* name, const float x, const float y, const float z) {
  const GLint loc = glGetUniformLocation(program, name); // Get uniform location by name
  glUniform3f(loc, x, y, z); // Set uniform to 3D vector (x, y, z)
}

void Program::setUniform(const char* name, const float& x, const float& y, const float& z, const float& w) {
  const GLint loc = glGetUniformLocation(program, name); // Get uniform location by name
  glUniform4f(loc, x, y, z, w); // Set uniform to 4D vector (x, y, z, w)
}

void Program::setUniform(const char* name, const Eigen::Matrix3f& matrix, const bool transpose) {
  const GLuint loc = glGetUniformLocation(program, name); // Get uniform location by name
  glUniformMatrix3fv(loc, 1, transpose, &matrix.coeff(0)); // Set uniform to 3x3 matrix
}

void Program::setUniform(const char* name, const Eigen::Matrix4f& matrix, const bool transpose) {
  const GLuint loc = glGetUniformLocation(program, name); // Get uniform location by name
  glUniformMatrix4fv(loc, 1, transpose, &matrix.coeff(0)); // Set uniform to 4x4 matrix
}

void Program::enableVertexAttribArray(const char* name) {
  const GLuint loc = glGetAttribLocation(program, name); // Get attribute location by name
  glEnableVertexAttribArray(loc); // Enable the vertex attribute array
}

void Program::disableVertexAttribArray(const char* name) {
  const GLuint loc = glGetAttribLocation(program, name); // Get attribute location by name
  glDisableVertexAttribArray(loc); // Disable the vertex attribute array
}

}
