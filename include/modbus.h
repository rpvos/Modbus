#ifndef MODBUS_H_
#define MODBUS_H_
#include <Arduino.h>
#include <crc16.h>

class ModbusSlave
{
public:
    ModbusSlave(int slave_id, int broadcast_id);

    void SetSlaveId(const uint8_t slave_id);
    const uint8_t GetSlaveId();

    const size_t GetReadMessageLength();
    const size_t GetReadResponseMessageLength(const word number_of_registers);
    const size_t GetWriteMessageLength(const word number_of_registers);
    const size_t GetWriteResponseMessageLength();

    void ConstructWriteMessage(byte *const message, const byte function_code, const word address, const word number_of_registers, const word *const register_values, const bool broadcast = false);
    bool ValidateWriteResponse(const byte *const message, const byte function_code, const word address, const word number_of_registers, const bool broadcast = false);

    void ConstructReadMessage(byte *const message, const byte function_code, const word address, const word number_of_registers, const bool broadcast = false);
    bool ValidateReadResponse(const byte *const message, const byte function_code, const word number_of_registers, const bool broadcast = false);
    void GetReadRegister(const byte *const message, const word number_of_registers, word *const register_values);

private:
    uint8_t slave_id_;
    uint8_t broadcast_id_;
};

#endif