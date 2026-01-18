#include "i2c_slave.h"
#include "cmd_response.h"

#include <QDebug>
#include <QThread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cstddef>
#include <errno.h>

I2cSlave::I2cSlave(const QString& device_path, uint8_t slave_address)
  : device_path_(device_path),
    slave_address_(slave_address),
    fd_(-1),
    is_open_(false),
    cmd_id_(0) {
}

I2cSlave::~I2cSlave() {
  close();
}

bool I2cSlave::open() {
  if (is_open_) {
    return true;
  }
  
  fd_ = ::open(device_path_.toLocal8Bit().constData(), O_RDWR);
  if (fd_ < 0) {
    qDebug() << "Failed to open I2C device:" << device_path_ << "-" << strerror(errno);
    return false;
  }
  
  if (ioctl(fd_, I2C_SLAVE, slave_address_) < 0) {
    qDebug() << "Failed to set I2C slave address:" << slave_address_ << "-" << strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  
  // Small delay to let I2C bus stabilize after opening
  usleep(10000);  // 10ms
  
  is_open_ = true;
  qDebug() << "I2C device opened:" << device_path_ << "at address" << slave_address_;
  return true;
}

void I2cSlave::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  is_open_ = false;
}

bool I2cSlave::isOpen() const {
  return is_open_;
}

/**
 * Write data at specified offset using PololuRPiSlave protocol
 * Protocol: [index byte] [data bytes...]
 * Limited to 16 bytes of data per transaction
 * 
 * Following Pololu's Python example pattern:
 * - After each write, add a 100us delay to allow AVR TWI module to process
 *   the STOP interrupt before any subsequent read operation
 * 
 * Includes retry logic for transient I2C errors (EIO, EAGAIN, etc.)
 */
bool I2cSlave::writeAtOffset(uint8_t offset, const uint8_t* data, size_t length) {
  if (!is_open_ || fd_ < 0) {
    return false;
  }
  
  // PololuRPiSlave limits writes to 16 bytes
  if (length > 16) {
    qDebug() << "I2C write truncated to 16 bytes (PololuRPiSlave limit)";
    length = 16;
  }
  
  // Prepare buffer: [index] [data...]
  uint8_t buffer[17];
  buffer[0] = offset;
  memcpy(&buffer[1], data, length);
  
  // Retry logic for transient I2C errors
  const int max_retries = 3;
  const int retry_delay_us = 1000;  // 1ms between retries
  
  for (int attempt = 0; attempt < max_retries; attempt++) {
    ssize_t written = ::write(fd_, buffer, length + 1);
    
    if (written == static_cast<ssize_t>(length + 1)) {
      // Success - add delay and return
      // Critical: Add 100us delay after write to allow AVR TWI module to process
      // STOP interrupt. Without this, quick write->read transitions can fail.
      // Reference: https://github.com/pololu/pololu-rpi-slave-arduino-library/blob/master/pi/a_star.py
      usleep(100);  // 100 microseconds = 0.0001 seconds
      return true;
    }
    
    // Handle errors
    if (written < 0) {
      int err = errno;
      // Retry on transient errors
      if ((err == EIO || err == EAGAIN || err == ETIMEDOUT) && attempt < max_retries - 1) {
        // Wait before retry
        usleep(retry_delay_us * (attempt + 1));  // Exponential backoff
        continue;
      }
      // Log error only on final attempt
      if (attempt == max_retries - 1) {
        qDebug() << "I2C write error after" << max_retries << "attempts:" << strerror(err);
      }
      return false;
    }
    
    // Partial write - shouldn't happen but handle it
    if (written != static_cast<ssize_t>(length + 1)) {
      qDebug() << "I2C write incomplete: expected" << (length + 1) << "wrote" << written;
      return false;
    }
  }
  
  return false;
}

/**
 * Read data at specified offset using PololuRPiSlave protocol
 * Protocol: Send [index byte], then read bytes starting from that index
 * 
 * Following Pololu's Python example pattern:
 * - Write the offset byte first
 * - Wait 100us to allow AVR TWI module to process STOP interrupt
 * - Then read the data bytes
 * 
 * This is more reliable than I2C_RDWR (combined write+read) because the AVR's
 * TWI module can't handle quick write->read transitions reliably.
 * Reference: https://github.com/pololu/pololu-rpi-slave-arduino-library/blob/master/pi/a_star.py
 * 
 * @param log_errors If true, log I2C errors. Set to false during polling to avoid spam.
 */
bool I2cSlave::readAtOffset(uint8_t offset, uint8_t* data, size_t length, bool log_errors) {
  if (!is_open_ || fd_ < 0) {
    return false;
  }
  
  // Step 1: Write the offset byte (sets the read position)
  // Retry logic for transient errors
  const int max_retries = 3;
  const int retry_delay_us = 1000;  // 1ms between retries
  
  ssize_t written = -1;
  for (int attempt = 0; attempt < max_retries; attempt++) {
    written = ::write(fd_, &offset, 1);
    if (written == 1) {
      break;  // Success
    }
    
    if (written < 0) {
      int err = errno;
      // Retry on transient errors
      if ((err == EIO || err == EAGAIN || err == ETIMEDOUT) && attempt < max_retries - 1) {
        usleep(retry_delay_us * (attempt + 1));
        continue;
      }
      if (log_errors && attempt == max_retries - 1) {
        qDebug() << "I2C write (offset) error after" << max_retries << "attempts:" << strerror(err);
      }
      return false;
    }
  }
  
  if (written != 1) {
    if (log_errors) {
      qDebug() << "I2C write (offset) incomplete: expected 1, wrote" << written;
    }
    return false;
  }
  
  // Step 2: Critical delay - allow AVR TWI module to process STOP interrupt
  // Without this, the read operation can fail because the TWI module is
  // disabled until the interrupt is processed.
  usleep(100);  // 100 microseconds = 0.0001 seconds
  
  // Step 3: Read the data bytes (with retry logic)
  for (int attempt = 0; attempt < max_retries; attempt++) {
    ssize_t bytes_read = ::read(fd_, data, length);
    
    if (bytes_read == static_cast<ssize_t>(length)) {
      return true;  // Success
    }
    
    if (bytes_read < 0) {
      int err = errno;
      // Retry on transient errors
      if ((err == EIO || err == EAGAIN || err == ETIMEDOUT) && attempt < max_retries - 1) {
        usleep(retry_delay_us * (attempt + 1));
        continue;
      }
      if (log_errors && attempt == max_retries - 1) {
        qDebug() << "I2C read error after" << max_retries << "attempts:" << strerror(err);
      }
      return false;
    }
    
    // Partial read
    if (log_errors && attempt == max_retries - 1) {
      qDebug() << "I2C read incomplete: expected" << length << "got" << bytes_read;
    }
    return false;
  }
  
  return false;
}

/**
 * Send a simple command (single character)
 * @param log_errors If false, suppress error logging (useful for cleanup commands)
 */
bool I2cSlave::sendCommand(char cmd, bool log_errors) {
  // Increment cmd_id before sending command
  ++cmd_id_;
  
  struct CommandData cmd_data;
  memset(&cmd_data, 0, sizeof(cmd_data));
  cmd_data.cmd = cmd;
  cmd_data.cmd_id = cmd_id_;
  
  return writeAtOffset(0, reinterpret_cast<const uint8_t*>(&cmd_data), sizeof(CommandData));
}

/**
 * Send waypoint command
 * Sends waypoint coordinates to Arduino
 */
bool I2cSlave::sendWaypointCommand(int32_t x, int32_t y) {
  // Note: writeAtOffset already includes 100us delay after write (per Pololu reference)
  
  // Increment cmd_id before sending command
  ++cmd_id_;
  
  struct CommandData cmd_data;
  memset(&cmd_data, 0, sizeof(cmd_data));
  cmd_data.cmd = 'w';
  cmd_data.cmd_id = cmd_id_;
  cmd_data.data.way_point.x = x;
  cmd_data.data.way_point.y = y;
  
  // Write CommandData (14 bytes: cmd + cmd_id + union) starting at offset 0
  if (!writeAtOffset(0, reinterpret_cast<const uint8_t*>(&cmd_data), sizeof(CommandData))) {
    qDebug() << "Failed to write waypoint command";
    return false;
  }
  
  // Verify the command was written by reading it back
  // Note: readAtOffset already includes 100us delay after writing offset (per Pololu reference)
  struct Data verify_buffer;
  memset(&verify_buffer, 0, sizeof(verify_buffer));
  
  // Verify the command was written (silent check)
  if (readAtOffset(0, reinterpret_cast<uint8_t*>(&verify_buffer), sizeof(struct Data), false)) {
    if (verify_buffer.cmd_data.cmd != 'w') {
      qDebug() << "Warning: Command in buffer is" << (char)verify_buffer.cmd_data.cmd << "not 'w'";
    }
  }
  
  // Poll for response with timeout
  // Arduino processes waypoint command synchronously, so response should be ready quickly
  struct Data full_buffer;
  memset(&full_buffer, 0, sizeof(full_buffer));
  
  const int timeout_us = 100000;  // 100ms timeout
  const int poll_interval_us = 100;  // Check every 100us (matching Pololu's pattern)
  int elapsed_us = 0;
  bool response_received = false;
  
  while (elapsed_us < timeout_us) {
    if (readAtOffset(0, reinterpret_cast<uint8_t*>(&full_buffer), sizeof(struct Data), false)) {
      // Check if Arduino echoed the waypoint back
      if (full_buffer.resp_data.cmd == 'w') {
        // Verify the coordinates match
        if (full_buffer.resp_data.data.way_point.x == x && 
            full_buffer.resp_data.data.way_point.y == y) {
          response_received = true;
          break;
        } else {
          qDebug() << "Waypoint response mismatch: expected x=" << x << "y=" << y
                   << "got x=" << full_buffer.resp_data.data.way_point.x 
                   << "y=" << full_buffer.resp_data.data.way_point.y;
        }
      }
    }
    usleep(poll_interval_us);
    elapsed_us += poll_interval_us;
  }
  
  if (!response_received) {
    qDebug() << "Warning: Waypoint command sent but no acknowledgment received after" 
             << (elapsed_us / 1000) << "ms";
    // Still return true - the command might have been processed even without response
  }
  
  return true;
}

/**
 * Send reset command
 */
bool I2cSlave::sendResetCommand() {
  return sendCommand('r');
}

/**
 * Get status from Arduino
 * Reads the entire Data buffer and extracts ResponseData from offset 14 (after CommandData)
 */
bool I2cSlave::getStatus(struct Status& status) {
  // Send status command
  if (!sendCommand('s')) {
    qDebug() << "Failed to send status command";
    return false;
  }
  
  // Small initial delay to let Arduino start processing the command
  // Arduino loop runs at 40Hz (25ms), but command processing is synchronous
  usleep(100);  // 10ms initial delay
  
  // Poll for response with timeout
  // Arduino continuously polls and processes commands synchronously, so response should be ready quickly
  const int timeout_us = 100000;  // Maximum time to wait: 100ms
  const int poll_interval_us = 50;  // Check every 0.05ms (50 microseconds)
  int elapsed_us = 100;  // Start with initial delay already elapsed
  int consecutive_errors = 0;
  const int max_consecutive_errors = 3;
  
  struct Data full_buffer;
  bool response_ready = false;
  
  while (elapsed_us < timeout_us) {
    memset(&full_buffer, 0, sizeof(full_buffer));
    
    // Don't log errors during polling to avoid spam
    if (!readAtOffset(0, reinterpret_cast<uint8_t*>(&full_buffer), sizeof(struct Data), false)) {
      // I2C read error - wait a bit and retry
      consecutive_errors++;
      if (consecutive_errors >= max_consecutive_errors) {
        qDebug() << "Too many consecutive I2C read errors (" << consecutive_errors 
                 << "), giving up after" << (elapsed_us / 1000) << "ms";
        return false;
      }
      usleep(poll_interval_us);
      elapsed_us += poll_interval_us;
      continue;
    }
    
    // Reset error counter on successful read
    consecutive_errors = 0;
    
    // Check if response is ready (cmd == 's')
    if (full_buffer.resp_data.cmd == 's') {
      response_ready = true;
      break;
    }
    
    // Response not ready yet, wait and retry
    usleep(poll_interval_us);
    elapsed_us += poll_interval_us;
  }
  
  if (!response_ready) {
    qDebug() << "Timeout waiting for status response after" << (elapsed_us / 1000) << "ms";
    return false;
  }
  
  status = full_buffer.resp_data.data.status;
  
  return true;
}

