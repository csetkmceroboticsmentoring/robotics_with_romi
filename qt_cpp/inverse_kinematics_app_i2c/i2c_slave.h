#pragma once

#include <QString>
#include <cstdint>

// Forward declarations
struct Status;
struct ResponseData;

/**
 * I2C Communication class for PololuRPiSlave protocol
 * 
 * The PololuRPiSlave library uses an index-based protocol:
 * - Write: Send [index byte] [data bytes...] (max 16 bytes per transaction)
 * - Read: Send [index byte], then read bytes starting from that index
 * 
 * The library manages a shared buffer where:
 * - Command data is written starting at offset 0
 * - Response data is read starting at offset 13 (after CommandData)
 */
class I2cSlave {
public:
  I2cSlave(const QString& device_path, uint8_t slave_address);
  ~I2cSlave();
  
  bool isOpen() const;
  bool open();
  void close();
  
  // High-level operations
  bool sendCommand(char cmd, bool log_errors = true);
  bool sendWaypointCommand(int32_t x, int32_t y);
  bool sendResetCommand();
  bool getStatus(struct Status& status);
  
private:
  // Low-level I2C operations (PololuRPiSlave protocol)
  bool writeAtOffset(uint8_t offset, const uint8_t* data, size_t length);
  bool readAtOffset(uint8_t offset, uint8_t* data, size_t length, bool log_errors = true);
  
  QString device_path_;
  uint8_t slave_address_;
  int fd_;
  bool is_open_;
  uint8_t cmd_id_;  // Command ID counter, incremented before each command
};

