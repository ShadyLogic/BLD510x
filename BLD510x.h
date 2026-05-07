#pragma once

#include <Arduino.h>

/**
 * @brief RS-485 / Modbus RTU driver for StepperOnline BLD-510B and BLD-510S drivers.
 *
 * The BLD-510B and BLD-510S share the same Modbus RTU register map for the
 * commands implemented here. They use standard Modbus RTU framing for register
 * addresses and CRC, but the speed command register uses little-endian value
 * bytes in the manual examples. This class keeps that device-specific quirk
 * isolated.
 */
class BLD510x
{
public:
    /**
     * @brief Motor rotation direction used by communication control commands.
     */
    enum Direction
    {
        Forward,
        Reverse
    };

    /**
     * @brief Fault bits reported by register 0x801B.
     */
    enum Fault : uint8_t
    {
        FaultNone             = 0x00,
        FaultLockedRotor      = 0x01,
        FaultOverCurrent      = 0x02,
        FaultHallAbnormal     = 0x04,
        FaultBusVoltageLow    = 0x08,
        FaultBusVoltageHigh   = 0x10,
        FaultCurrentPeakAlarm = 0x20
    };

    /**
     * @brief Creates a BLD-510x driver instance.
     *
     * @param serial Stream connected to the RS-485 transceiver. The sketch must
     * call begin() on the serial object before using this driver.
     * @param deRePin Digital pin controlling the transceiver DE/RE direction.
     * @param slaveAddress Initial Modbus slave address, valid range 1-250.
     */
    BLD510x(Stream &serial, uint8_t deRePin, uint8_t slaveAddress = 1);

    /**
     * @brief Initializes the RS-485 direction pin.
     *
     * The sketch is responsible for initializing the serial object passed to
     * the constructor. For hardware serial, use settings compatible with the
     * driver, typically 9600 baud, 8 data bits, no parity, and 1 stop bit.
     */
    void begin();

    /**
     * @brief Sets the local Modbus target address used for future commands.
     *
     * This does not write the address into the motor driver. Use
     * writeDeviceAddress() to change the driver's configured address register.
     *
     * @param address Modbus slave address, valid range 1-250.
     * @return true if the address is valid.
     */
    bool setAddress(uint8_t address);

    /**
     * @brief Sets the local Modbus target address used for future commands.
     *
     * This is an explicit alias for setAddress().
     *
     * @param address Modbus slave address, valid range 1-250.
     * @return true if the address is valid.
     */
    bool useAddress(uint8_t address);

    /**
     * @brief Writes a new Modbus address to the BLD-510B address register.
     *
     * This is a device configuration command. After a successful write, the
     * local target address is updated to the new address.
     *
     * @param address New device address, valid range 1-250.
     * @return true if the device acknowledged the write.
     */
    bool writeDeviceAddress(uint8_t address);

    /**
     * @brief Sets the motor pole-pair count used in control words and RPM math.
     *
     * The BLD-510B register stores pole pairs, while speed feedback conversion
     * uses motor poles. getRPM() converts pole pairs to poles internally.
     *
     * @param polePairs Pole-pair count, valid range 1-15.
     * @return true if the value is valid.
     */
    bool setPolePairs(uint8_t polePairs);

    /**
     * @brief Sets the serial response timeout for Modbus replies.
     *
     * @param timeoutMs Timeout in milliseconds. Zero is ignored.
     */
    void setResponseTimeout(uint32_t timeoutMs);

    /**
     * @brief Enables or disables hex logging of transmitted and received frames.
     *
     * Pass nullptr to disable logging.
     *
     * @param debugStream Stream to receive debug output, such as &Serial.
     */
    void setDebugStream(Stream *debugStream);

    /**
     * @brief Enables communication control and starts the motor.
     *
     * @param direction Forward or Reverse.
     * @return true if the driver acknowledged the command.
     */
    bool enable(Direction direction = Forward);

    /**
     * @brief Performs a natural stop while leaving communication control active.
     *
     * @param direction Direction bit to preserve in the control word.
     * @return true if the driver acknowledged the command.
     */
    bool stop(Direction direction = Forward);

    /**
     * @brief Performs a braking stop.
     *
     * The command matches the manual's braking-stop example: NW, BK, and EN set.
     *
     * @return true if the driver acknowledged the command.
     */
    bool brake();

    /**
     * @brief Sets closed-loop speed in RPM.
     *
     * Kept for compatibility. Prefer setSpeedRpm() for clearer intent.
     *
     * @param speedValue Target speed in RPM.
     * @return true if the driver acknowledged the command.
     */
    bool setSpeed(uint16_t speedValue);

    /**
     * @brief Sets closed-loop target speed in RPM.
     *
     * Register 0x8005 is written using little-endian value bytes, matching the
     * BLD-510B manual examples for 1000 RPM and 1500 RPM.
     *
     * @param rpm Target speed in RPM.
     * @return true if the driver acknowledged the command.
     */
    bool setSpeedRpm(uint16_t rpm);

    /**
     * @brief Sets acceleration and deceleration times.
     *
     * @param accelTenths Acceleration time in tenths of a second.
     * @param decelTenths Deceleration time in tenths of a second.
     * @return true if the driver acknowledged the command.
     */
    bool setAccelerationDeceleration(uint8_t accelTenths, uint8_t decelTenths);

    /**
     * @brief Reads the raw actual-speed register value from 0x8018.
     *
     * Convert to RPM with raw * 20 / motorPoles.
     *
     * @param value Receives the raw register value.
     * @return true if the read completed and passed CRC validation.
     */
    bool readActualSpeedRaw(uint16_t &value);

    /**
     * @brief Reads actual speed and converts it to RPM.
     *
     * @param rpm Receives the converted RPM value.
     * @param motorPoles Total motor pole count, not pole pairs.
     * @return true if the read completed and motorPoles is nonzero.
     */
    bool readActualSpeedRpm(float &rpm, uint8_t motorPoles);

    /**
     * @brief Reads the fault bits from register 0x801B.
     *
     * @param faultBits Receives a bitmask from the Fault enum.
     * @return true if the read completed and passed CRC validation.
     */
    bool readFault(uint8_t &faultBits);

    /**
     * @brief Clears an alarm by issuing a natural stop command.
     *
     * @return true if the stop command was acknowledged.
     */
    bool clearAlarmByStop();

    /**
     * @brief Convenience wrapper for enable(Forward).
     */
    bool forward();

    /**
     * @brief Convenience wrapper for enable(Reverse).
     */
    bool reverse();

    /**
     * @brief Sets target RPM, then enables the motor in the requested direction.
     *
     * @param rpm Target speed in RPM. Clamped to 0 through getMaxRPM().
     * @param direction Forward or Reverse.
     * @return true if both commands were acknowledged.
     */
    bool runRPM(float rpm, Direction direction = Forward);

    /**
     * @brief Sets target RPM after clamping to the configured maximum.
     *
     * @param rpm Target speed in RPM.
     * @return true if the driver acknowledged the command.
     */
    bool setRPM(float rpm);

    /**
     * @brief Performs a natural stop.
     */
    bool coast();

    /**
     * @brief Performs a braking stop.
     */
    bool emergencyBrake();

    /**
     * @brief Reads actual speed using the configured pole-pair count.
     *
     * @param rpm Receives the converted RPM value.
     * @return true if the speed read succeeded.
     */
    bool getRPM(float &rpm);

    /**
     * @brief Checks whether any fault bit is currently set.
     *
     * Returns true if the fault register cannot be read, so callers fail toward
     * the safer path.
     *
     * @return true when a fault is present or the read fails.
     */
    bool hasFault();

    /**
     * @brief Prints the current fault state to a stream.
     *
     * @param out Destination stream.
     * @return true if the fault register was read successfully.
     */
    bool printFault(Stream &out);

    /**
     * @brief Sets the maximum RPM used by setRPM() and runRPM() clamping.
     *
     * @param maxRPM New maximum RPM. Values less than or equal to zero are ignored.
     */
    void setMaxRPM(float maxRPM);

    /**
     * @brief Returns the configured maximum RPM clamp.
     */
    float getMaxRPM() const;

private:
    Stream *_serial;
    Stream *_debugStream;
    float _maxRPM = 7000.0f;
    uint32_t _responseTimeoutMs;
    uint8_t _deRePin;
    uint8_t _slaveAddress;
    uint8_t _polePairs;

    static constexpr uint8_t FUNC_READ_HOLDING_REGISTERS = 0x03;
    static constexpr uint8_t FUNC_WRITE_SINGLE_REGISTER = 0x06;
    static constexpr uint8_t MODBUS_EXCEPTION_FLAG = 0x80;

    static constexpr uint8_t CONTROL_EN = 0x01;
    static constexpr uint8_t CONTROL_FR = 0x02;
    static constexpr uint8_t CONTROL_BK = 0x04;
    static constexpr uint8_t CONTROL_NW = 0x08;

    static constexpr uint16_t REG_CONTROL = 0x8000;
    static constexpr uint16_t REG_ACCEL_DECEL = 0x8003;
    static constexpr uint16_t REG_SPEED = 0x8005;
    static constexpr uint16_t REG_ADDRESS = 0x8007;
    static constexpr uint16_t REG_ACTUAL_SPEED = 0x8018;
    static constexpr uint16_t REG_FAULT = 0x801B;

    bool writeRegister(uint16_t reg, uint16_t value);

    /**
     * @brief Writes a register whose value field is sent low byte first.
     *
     * This is intentionally not normal Modbus word order. The BLD-510B manual's
     * speed examples show 1000 RPM as E8 03 and 1500 RPM as DC 05.
     */
    bool writeRegisterLittleEndianValue(uint16_t reg, uint16_t value);
    bool readRegisters(uint16_t reg, uint16_t count, uint16_t *values);

    bool sendFrame(const uint8_t *frame, size_t length);
    bool readResponse(uint8_t *buffer, size_t expectedLength, uint32_t timeoutMs = 100);

    uint16_t crc16(const uint8_t *data, size_t length);
    void setTransmit(bool enabled);
    void logFrame(const char *label, const uint8_t *frame, size_t length);
    void printHexByte(uint8_t value);

    uint16_t makeControlWord(uint8_t controlBits);
};
