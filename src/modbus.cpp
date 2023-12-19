#include "modbus.h"

const uint8_t kSizeOfByte = 8;

// Common indexes for message
const uint8_t kSlaveIdMessageIndex = 0;
const uint8_t kFunctionCodeMessageIndex = 1;
const uint8_t kAddressUpperMessageIndex = 2;
const uint8_t kAddressLowerMessageIndex = 3;
const uint8_t kNumberOfRegistersUpperMessageIndex = 4;
const uint8_t kNumberOfRegistersLowerMessageIndex = 5;

// Write message indexes
const uint8_t kWriteDataLengthMessageIndex = 6;
const uint8_t kWriteRegisterValueUpperMessageIndex = 7;
const uint8_t kWriteRegisterValueLowerMessageIndex = 8;

// Write response message indexes
const uint8_t kWriteResponseAddressUpperMessageIndex = 2;
const uint8_t kWriteResponseAddressLowerMessageIndex = 3;
const uint8_t kWriteResponseNumberOfRegistersUpperMessageIndex = 4;
const uint8_t kWriteResponseNumberOfRegistersLowerMessageIndex = 5;
const uint8_t kWriteResponseMessageLength = 8;

// Read message length
const uint8_t kReadMessageLength = 8;

// Read response message indexes
const uint8_t kReadResponseDataLengthMessageIndex = 2;
const uint8_t kReadResponseRegisterValueUpperMessageIndex = 3;
const uint8_t kReadResponseRegisterValueLowerMessageIndex = 4;

// Crc length used for all messages
const uint8_t kCrcLength = 2;

ModbusSlave::ModbusSlave(int slave_id, int broadcast_id)
{
    this->slave_id_ = slave_id;
    this->broadcast_id_ = broadcast_id;
}

void ModbusSlave::SetSlaveId(const uint8_t slave_id)
{
    this->slave_id_ = slave_id;
}

const uint8_t ModbusSlave::GetSlaveId()
{
    return this->slave_id_;
}

const size_t ModbusSlave::GetReadMessageLength()
{
    return kReadMessageLength;
}

const size_t ModbusSlave::GetReadResponseMessageLength(const word number_of_registers)
{
    size_t data_length = (2 * number_of_registers) + 1;
    return kReadResponseDataLengthMessageIndex + data_length + kCrcLength;
}

const size_t ModbusSlave::GetWriteMessageLength(const word number_of_registers)
{
    size_t data_length = (2 * number_of_registers) + 1;
    return size_t(kWriteDataLengthMessageIndex + data_length + kCrcLength);
}

const size_t ModbusSlave::GetWriteResponseMessageLength()
{
    return kWriteResponseMessageLength;
}

void ModbusSlave::ConstructWriteMessage(byte *const message, const byte function_code, const word address, const word number_of_registers, const word *register_values, const bool broadcast)
{
    if (broadcast)
    {
        message[kSlaveIdMessageIndex] = this->broadcast_id_;
    }
    else
    {
        message[kSlaveIdMessageIndex] = this->slave_id_;
    }

    // Compute every byte of message
    const byte address_upper = address >> kSizeOfByte & 0xFF;
    const byte address_lower = address & 0xFF;
    const byte number_of_registers_upper = number_of_registers >> kSizeOfByte & 0xFF;
    const byte number_of_registers_lower = number_of_registers & 0xFF;
    const byte data_length = 2 * number_of_registers;

    message[kFunctionCodeMessageIndex] = function_code;
    message[kAddressUpperMessageIndex] = address_upper;
    message[kAddressLowerMessageIndex] = address_lower;
    message[kNumberOfRegistersUpperMessageIndex] = number_of_registers_upper;
    message[kNumberOfRegistersLowerMessageIndex] = number_of_registers_lower;
    message[kWriteDataLengthMessageIndex] = data_length;

    for (size_t register_number = 0; register_number < number_of_registers; register_number++)
    {
        byte upper_register_value = register_values[register_number] >> kSizeOfByte & 0xFF;
        byte lower_register_value = register_values[register_number] & 0xFF;

        message[kWriteRegisterValueUpperMessageIndex + (register_number * 2)] = upper_register_value;
        message[kWriteRegisterValueLowerMessageIndex + (register_number * 2)] = lower_register_value;
    }

    Crc::AddCrcModbus(message, message, GetWriteMessageLength(number_of_registers) - kCrcLength, false);
}

bool ModbusSlave::ValidateWriteResponse(const byte *const message, const byte function_code, const word address, const word number_of_registers, const bool broadcast)
{
    if (broadcast)
    {
        // slave number 0 is broadcast to all connected devices
        if (message[kSlaveIdMessageIndex] != this->broadcast_id_)
        {
            return false;
        }
    }
    else
    {
        if (message[kSlaveIdMessageIndex] != this->slave_id_)
        {
            return false;
        };
    }

    if (message[kFunctionCodeMessageIndex] != function_code)
    {
        return false;
    };

    const byte address_upper = address >> kSizeOfByte & 0xFF;
    const byte address_lower = address & 0xFF;

    if (message[kAddressUpperMessageIndex] != address_upper)
    {
        return false;
    };
    if (message[kAddressLowerMessageIndex] != address_lower)
    {
        return false;
    };

    const byte number_of_registers_upper = number_of_registers >> kSizeOfByte & 0xFF;
    const byte number_of_registers_lower = number_of_registers & 0xFF;

    if (message[kNumberOfRegistersUpperMessageIndex] != number_of_registers_upper)
    {
        return false;
    };
    if (message[kNumberOfRegistersLowerMessageIndex] != number_of_registers_lower)
    {
        return false;
    };

    return Crc::ValidateCrcModbus(message, GetWriteResponseMessageLength(), false);
}

void ModbusSlave::ConstructReadMessage(byte *const message, const byte function_code, const word address, const word number_of_registers, const bool broadcast)
{
    if (broadcast)
    {
        message[kSlaveIdMessageIndex] = this->broadcast_id_;
    }
    else
    {
        message[kSlaveIdMessageIndex] = this->slave_id_;
    }

    // Compute every byte of message
    const byte address_upper = address >> kSizeOfByte & 0xFF;
    const byte address_lower = address & 0xFF;
    const byte number_of_registers_upper = number_of_registers >> kSizeOfByte & 0xFF;
    const byte number_of_registers_lower = number_of_registers & 0xFF;

    message[kFunctionCodeMessageIndex] = function_code;
    message[kAddressUpperMessageIndex] = address_upper;
    message[kAddressLowerMessageIndex] = address_lower;
    message[kNumberOfRegistersUpperMessageIndex] = number_of_registers_upper;
    message[kNumberOfRegistersLowerMessageIndex] = number_of_registers_lower;

    Crc::AddCrcModbus(message, message, GetReadMessageLength() - kCrcLength, false);
}

bool ModbusSlave::ValidateReadResponse(const byte *const message, const byte function_code, const word number_of_registers, const bool broadcast)
{
    if (broadcast)
    {
        // slave number 0 is broadcast to all connected devices
        if (message[kSlaveIdMessageIndex] != this->broadcast_id_)
        {
            return false;
        }
    }
    else
    {
        if (message[kSlaveIdMessageIndex] != this->slave_id_)
        {
            return false;
        };
    }

    if (message[kFunctionCodeMessageIndex] != function_code)
    {
        return false;
    };

    size_t data_length = number_of_registers * 2;

    if (message[kReadResponseDataLengthMessageIndex] != data_length)
    {
        return false;
    };

    return Crc::ValidateCrcModbus(message, GetReadResponseMessageLength(number_of_registers), false);
}

void ModbusSlave::GetReadRegisters(const byte *const message, const word number_of_registers, word *const register_value)
{
    for (size_t i = 0; i < number_of_registers; i++)
    {
        register_value[i] = message[kReadResponseRegisterValueUpperMessageIndex + (i * 2)] << kSizeOfByte ||
                            message[kReadResponseRegisterValueLowerMessageIndex + (i * 2)];
    }
}
