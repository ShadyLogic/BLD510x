#include "BLD510x.h"

BLD510x::BLD510x(HardwareSerial &serial, uint8_t deRePin, uint8_t slaveAddress)
{
    _serial = &serial;
    _debugStream = nullptr;
    _deRePin = deRePin;
    _slaveAddress = slaveAddress;
    _polePairs = 2;
    _maxRPM = 7000.0f;
    _responseTimeoutMs = 100;
}

void BLD510x::begin(uint32_t baud)
{
    pinMode(_deRePin, OUTPUT);
    setTransmit(false);

    _serial->begin(baud, SERIAL_8N1);
}

bool BLD510x::setAddress(uint8_t address)
{
    return useAddress(address);
}

bool BLD510x::useAddress(uint8_t address)
{
    if (address < 1 || address > 250)
    {
        return false;
    }

    _slaveAddress = address;
    return true;
}

bool BLD510x::writeDeviceAddress(uint8_t address)
{
    if (address < 1 || address > 250)
    {
        return false;
    }

    if (!writeRegister(REG_ADDRESS, (uint16_t)address << 8))
    {
        return false;
    }

    _slaveAddress = address;
    return true;
}

bool BLD510x::setPolePairs(uint8_t polePairs)
{
    if (polePairs < 1 || polePairs > 15)
    {
        return false;
    }

    _polePairs = polePairs;
    return true;
}

void BLD510x::setResponseTimeout(uint32_t timeoutMs)
{
    if (timeoutMs > 0)
    {
        _responseTimeoutMs = timeoutMs;
    }
}

void BLD510x::setDebugStream(Stream *debugStream)
{
    _debugStream = debugStream;
}

bool BLD510x::enable(Direction direction)
{
    uint8_t bits = CONTROL_NW | CONTROL_EN;

    if (direction == Reverse)
    {
        bits |= CONTROL_FR;
    }

    return writeRegister(REG_CONTROL, makeControlWord(bits));
}

bool BLD510x::stop(Direction direction)
{
    uint8_t bits = CONTROL_NW;

    if (direction == Reverse)
    {
        bits |= CONTROL_FR;
    }

    return writeRegister(REG_CONTROL, makeControlWord(bits));
}

bool BLD510x::brake()
{
    return writeRegister(REG_CONTROL, makeControlWord(CONTROL_NW | CONTROL_BK | CONTROL_EN));
}

bool BLD510x::setSpeed(uint16_t speedValue)
{
    return setSpeedRpm(speedValue);
}

bool BLD510x::setSpeedRpm(uint16_t rpm)
{
    return writeRegisterLittleEndianValue(REG_SPEED, rpm);
}

bool BLD510x::setAccelerationDeceleration(uint8_t accelTenths, uint8_t decelTenths)
{
    uint16_t value = ((uint16_t)accelTenths << 8) | decelTenths;
    return writeRegister(REG_ACCEL_DECEL, value);
}

bool BLD510x::readActualSpeedRaw(uint16_t &value)
{
    if (!readRegisters(REG_ACTUAL_SPEED, 1, &value))
    {
        return false;
    }

    return true;
}

bool BLD510x::readActualSpeedRpm(float &rpm, uint8_t motorPoles)
{
    if (motorPoles == 0)
    {
        return false;
    }

    uint16_t raw = 0;

    if (!readActualSpeedRaw(raw))
    {
        return false;
    }

    rpm = ((float)raw * 20.0f) / (float)motorPoles;
    return true;
}

bool BLD510x::readFault(uint8_t &faultBits)
{
    uint16_t value = 0;

    if (!readRegisters(REG_FAULT, 1, &value))
    {
        return false;
    }

    faultBits = (uint8_t)(value >> 8);
    return true;
}

bool BLD510x::clearAlarmByStop()
{
    return stop(Forward);
}

bool BLD510x::forward()
{
    return enable(Forward);
}

bool BLD510x::reverse()
{
    return enable(Reverse);
}

bool BLD510x::runRPM(float rpm, Direction direction)
{
    if (!setRPM(rpm))
    {
        return false;
    }

    return enable(direction);
}

bool BLD510x::setRPM(float rpm)
{
    if (rpm < 0.0f)
    {
        rpm = 0.0f;
    }

    if (rpm > _maxRPM)
    {
        rpm = _maxRPM;
    }

    return setSpeedRpm((uint16_t)rpm);
}

bool BLD510x::coast()
{
    return stop(Forward);
}

bool BLD510x::emergencyBrake()
{
    return brake();
}

bool BLD510x::getRPM(float &rpm)
{
    uint16_t raw = 0;

    if (!readActualSpeedRaw(raw))
    {
        return false;
    }

    uint8_t motorPoles = _polePairs * 2;

    rpm = ((float)raw * 20.0f) / (float)motorPoles;
    return true;
}

bool BLD510x::hasFault()
{
    uint8_t fault = 0;

    if (!readFault(fault))
    {
        return true;
    }

    return fault != FaultNone;
}

bool BLD510x::printFault(Stream &out)
{
    uint8_t fault = 0;

    if (!readFault(fault))
    {
        out.println("Failed to read fault register");
        return false;
    }

    if (fault == FaultNone)
    {
        out.println("No fault");
        return true;
    }

    out.print("Fault bits: 0x");
    out.println(fault, HEX);

    if (fault & FaultLockedRotor)
    {
        out.println("Locked rotor");
    }

    if (fault & FaultOverCurrent)
    {
        out.println("Over-current");
    }

    if (fault & FaultHallAbnormal)
    {
        out.println("Hall signal abnormal");
    }

    if (fault & FaultBusVoltageLow)
    {
        out.println("Bus voltage too low");
    }

    if (fault & FaultBusVoltageHigh)
    {
        out.println("Bus voltage too high");
    }

    if (fault & FaultCurrentPeakAlarm)
    {
        out.println("Current peak alarm");
    }

    return true;
}

void BLD510x::setMaxRPM(float maxRPM)
{
    if (maxRPM > 0.0f)
    {
        _maxRPM = maxRPM;
    }
}

float BLD510x::getMaxRPM() const
{
    return _maxRPM;
}

uint16_t BLD510x::makeControlWord(uint8_t controlBits)
{
    return ((uint16_t)controlBits << 8) | (_polePairs & 0x0F);
}

bool BLD510x::writeRegister(uint16_t reg, uint16_t value)
{
    uint8_t frame[8];

    frame[0] = _slaveAddress;
    frame[1] = FUNC_WRITE_SINGLE_REGISTER;
    frame[2] = reg >> 8;
    frame[3] = reg & 0xFF;
    frame[4] = value >> 8;
    frame[5] = value & 0xFF;

    uint16_t crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    if (!sendFrame(frame, sizeof(frame)))
    {
        return false;
    }

    uint8_t response[8];

    if (!readResponse(response, sizeof(response), _responseTimeoutMs))
    {
        return false;
    }

    for (size_t i = 0; i < sizeof(frame); i++)
    {
        if (response[i] != frame[i])
        {
            return false;
        }
    }

    return true;
}

bool BLD510x::writeRegisterLittleEndianValue(uint16_t reg, uint16_t value)
{
    uint8_t frame[8];

    frame[0] = _slaveAddress;
    frame[1] = FUNC_WRITE_SINGLE_REGISTER;
    frame[2] = reg >> 8;
    frame[3] = reg & 0xFF;

    // BLD-510B speed write examples send 1000 RPM as E8 03 and 1500 RPM as DC 05.
    frame[4] = value & 0xFF;
    frame[5] = value >> 8;

    uint16_t crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    if (!sendFrame(frame, sizeof(frame)))
    {
        return false;
    }

    uint8_t response[8];

    if (!readResponse(response, sizeof(response), _responseTimeoutMs))
    {
        return false;
    }

    for (size_t i = 0; i < sizeof(frame); i++)
    {
        if (response[i] != frame[i])
        {
            return false;
        }
    }

    return true;
}

bool BLD510x::readRegisters(uint16_t reg, uint16_t count, uint16_t *values)
{
    if (count == 0 || count > 8)
    {
        return false;
    }

    uint8_t frame[8];

    frame[0] = _slaveAddress;
    frame[1] = FUNC_READ_HOLDING_REGISTERS;
    frame[2] = reg >> 8;
    frame[3] = reg & 0xFF;
    frame[4] = count >> 8;
    frame[5] = count & 0xFF;

    uint16_t crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    if (!sendFrame(frame, sizeof(frame)))
    {
        return false;
    }

    size_t expectedLength = 5 + (count * 2);
    uint8_t response[32];

    if (!readResponse(response, expectedLength, _responseTimeoutMs))
    {
        return false;
    }

    if (response[0] != _slaveAddress || response[1] != FUNC_READ_HOLDING_REGISTERS || response[2] != count * 2)
    {
        return false;
    }

    uint16_t receivedCrc = response[expectedLength - 2] | ((uint16_t)response[expectedLength - 1] << 8);
    uint16_t calculatedCrc = crc16(response, expectedLength - 2);

    if (receivedCrc != calculatedCrc)
    {
        return false;
    }

    for (uint16_t i = 0; i < count; i++)
    {
        values[i] = ((uint16_t)response[3 + i * 2] << 8) | response[4 + i * 2];
    }

    return true;
}

bool BLD510x::sendFrame(const uint8_t *frame, size_t length)
{
    while (_serial->available())
    {
        _serial->read();
    }

    setTransmit(true);

    logFrame("TX", frame, length);

    _serial->write(frame, length);
    _serial->flush();

    setTransmit(false);

    return true;
}

bool BLD510x::readResponse(uint8_t *buffer, size_t expectedLength, uint32_t timeoutMs)
{
    size_t index = 0;
    uint32_t start = millis();

    while ((millis() - start) < timeoutMs)
    {
        while (_serial->available())
        {
            if (index < expectedLength)
            {
                buffer[index++] = _serial->read();
            }
            else
            {
                _serial->read();
            }

            if (index >= expectedLength)
            {
                logFrame("RX", buffer, index);
                return true;
            }

            if (index == 5 && buffer[1] >= MODBUS_EXCEPTION_FLAG)
            {
                uint16_t receivedCrc = buffer[3] | ((uint16_t)buffer[4] << 8);

                if (receivedCrc == crc16(buffer, 3))
                {
                    logFrame("RX", buffer, index);
                    return true;
                }
            }
        }
    }

    if (index > 0)
    {
        logFrame("RX timeout", buffer, index);
    }

    return false;
}

uint16_t BLD510x::crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void BLD510x::setTransmit(bool enabled)
{
    digitalWrite(_deRePin, enabled ? HIGH : LOW);
}

void BLD510x::logFrame(const char *label, const uint8_t *frame, size_t length)
{
    if (_debugStream == nullptr)
    {
        return;
    }

    _debugStream->print(label);
    _debugStream->print(": ");

    for (size_t i = 0; i < length; i++)
    {
        if (i > 0)
        {
            _debugStream->print(' ');
        }

        printHexByte(frame[i]);
    }

    _debugStream->println();
}

void BLD510x::printHexByte(uint8_t value)
{
    if (value < 0x10)
    {
        _debugStream->print('0');
    }

    _debugStream->print(value, HEX);
}
