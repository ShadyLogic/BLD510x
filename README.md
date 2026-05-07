# BLD510x

Arduino / Teensy RS-485 Modbus RTU driver for StepperOnline BLD-510B and BLD-510S BLDC motor controllers.

## Features

- Supports StepperOnline BLD-510B and BLD-510S drivers.
- Uses Modbus RTU over RS-485 with any Arduino `Stream` object, including hardware serial and software serial.
- Handles the BLD-510x speed-register byte-order quirk from the manual examples.
- Supports run, stop, braking stop, direction, speed, acceleration/deceleration, speed feedback, address selection, and fault reads.
- Optional TX/RX frame logging for bring-up and troubleshooting.

## Hardware

You need an RS-485 transceiver between the microcontroller serial port and the motor driver. Pass the serial object and transceiver DE/RE direction-control pin to the constructor.

Typical wiring:

- MCU TX -> RS-485 transceiver DI
- MCU RX -> RS-485 transceiver RO
- MCU direction pin -> transceiver DE and /RE control
- Transceiver A/B -> BLD-510x RS-485 A/B
- Common ground between controller and driver, unless your interface is intentionally isolated

Initialize the serial object in your sketch before calling `motor.begin()`. Default driver serial settings are:

- 9600 baud
- 8 data bits
- no parity
- 1 stop bit

## Installation

### Arduino IDE

Clone or download this repository into your Arduino libraries folder:

```text
Documents/Arduino/libraries/BLD510x
```

Restart the Arduino IDE, then open:

```text
File > Examples > BLD510x > BLD510x_example
```

### PlatformIO

Use the repository as a library dependency:

```ini
lib_deps =
    https://github.com/ShadyLogic/BLD510x.git
```

Or copy this repository into your project's `lib/BLD510x` folder.

## Basic Usage

```cpp
#include <BLD510x.h>

constexpr uint8_t RS485_DE_RE_PIN = 2;

BLD510x motor(Serial1, RS485_DE_RE_PIN, 1);

void setup()
{
    Serial.begin(115200);

    Serial1.begin(9600, SERIAL_8N1);
    motor.begin();
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

Supported commands/registers include control, speed setting, acceleration/deceleration, driver address, actual speed feedback, and fault state.

## Debugging

Enable frame logging when wiring or verifying RS-485 communication:

```cpp
motor.setDebugStream(&Serial);
```

This prints transmitted and received Modbus frames in hex.

## Examples

See [`examples/BLD510x_example/BLD510x_example.ino`](examples/BLD510x_example/BLD510x_example.ino) for a fuller bring-up sketch that demonstrates:

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

## Safety

Test unloaded first. Verify motor direction, stop/brake behavior, fault handling, and any external emergency-stop hardware before attaching the motor to mechanics. A serial command or wiring mistake can start motion unexpectedly.

## References

- [BLD-510B product page](https://www.omc-stepperonline.com/digital-brushless-dc-motor-driver-12v-48vdc-max-15-0a-400w-bld-510b)
- [BLD-510S product page](https://www.stepperonline.ca/digital-brushless-dc-motor-driver-18v-50vdc-max-10a-200w-bld-510s.html)
- [BLD-510B manual mirror](https://www.manualslib.com/manual/3533809/Stepperonline-Bld-510b.html)
- [BLD-510S manual mirror](https://www.manualslib.com/manual/3284533/Stepperonline-Bld-510s.html)

## License

MIT License. See [`LICENSE`](LICENSE).
