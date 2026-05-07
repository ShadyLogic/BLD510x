# BLD510x

Arduino / Teensy RS-485 Modbus RTU driver for StepperOnline BLD-510B and BLD-510S BLDC motor controllers.

## Features

- Supports StepperOnline BLD-510B and BLD-510S drivers.
- Uses Modbus RTU over RS-485 with Arduino `HardwareSerial`.
- Handles the BLD-510x speed-register byte-order quirk from the manual examples.
- Supports run, stop, braking stop, direction, speed, acceleration/deceleration, speed feedback, address selection, and fault reads.
- Optional TX/RX frame logging for bring-up and troubleshooting.

## Hardware

You need an RS-485 transceiver between the microcontroller UART and the motor driver. Pass the transceiver DE/RE direction-control pin to the constructor.

Default serial settings are:

- 9600 baud
- 8 data bits
- no parity
- 1 stop bit

## Basic Usage

```cpp
#include <BLD510x.h>

constexpr uint8_t RS485_DE_RE_PIN = 2;

BLD510x motor(Serial1, RS485_DE_RE_PIN, 1);

void setup()
{
    Serial.begin(115200);

    motor.begin(9600);
    motor.setResponseTimeout(150);
    motor.setPolePairs(2);
    motor.setMaxRPM(7000.0f);
    motor.setAccelerationDeceleration(10, 10);

    motor.runRPM(1500.0f, BLD510x::Forward);
}

void loop()
{
    float rpm = 0.0f;

    if (motor.getRPM(rpm))
    {
        Serial.print("Motor RPM: ");
        Serial.println(rpm);
    }

    if (motor.hasFault())
    {
        motor.printFault(Serial);
        motor.emergencyBrake();
    }

    delay(500);
}
```

## Supported Drivers

The BLD-510B and BLD-510S use the same Modbus command structure for the functions implemented by this library.

## Debugging

Enable frame logging when wiring or verifying RS-485 communication:

```cpp
motor.setDebugStream(&Serial);
```

This prints transmitted and received Modbus frames in hex.

## Examples

See [`examples/BLD510x_example.cpp`](examples/BLD510x_example.cpp) for a fuller bring-up sketch that demonstrates:

- setup command checks
- response timeout configuration
- optional frame logging
- forward/reverse run
- speed changes
- natural stop
- braking stop
- speed feedback
- fault handling

## Notes

The speed command register is intentionally written with little-endian value bytes because the BLD-510B and BLD-510S manual examples show values such as 1000 RPM as `E8 03` and 1500 RPM as `DC 05`. Register addresses and CRC bytes still follow normal Modbus RTU ordering.

## License

MIT License. See [`LICENSE`](LICENSE).
