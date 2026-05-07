#include "BLD510x.h"

constexpr uint8_t RS485_DE_RE_PIN = 2; // Change to match your RS-485 DE/RE wiring.
constexpr uint8_t MOTOR_ADDRESS = 1;
constexpr uint8_t MOTOR_POLE_PAIRS = 2;

constexpr uint32_t PC_BAUD = 115200;
constexpr uint32_t MOTOR_BAUD = 9600;
constexpr uint32_t RESPONSE_TIMEOUT_MS = 150;

constexpr uint16_t FORWARD_RPM = 1500;
constexpr uint16_t FASTER_RPM = 2500;
constexpr uint16_t REVERSE_RPM = 1000;

BLD510x motor(Serial1, RS485_DE_RE_PIN, MOTOR_ADDRESS);

enum DemoStep
{
    DemoStartForward,
    DemoSpeedUp,
    DemoCoast,
    DemoReverse,
    DemoBrake,
    DemoDone
};

DemoStep demoStep = DemoStartForward;
uint32_t nextStepAtMs = 0;

bool checked(const char *label, bool ok)
{
    Serial.print(label);
    Serial.println(ok ? " OK" : " FAILED");
    return ok;
}

void scheduleNext(DemoStep nextStep, uint32_t delayMs)
{
    demoStep = nextStep;
    nextStepAtMs = millis() + delayMs;
}

void runDemoStep()
{
    if ((int32_t)(millis() - nextStepAtMs) < 0)
    {
        return;
    }

    switch (demoStep)
    {
        case DemoStartForward:
            Serial.println("Forward run");
            checked("runRPM forward", motor.runRPM(FORWARD_RPM, BLD510x::Forward));
            scheduleNext(DemoSpeedUp, 5000);
            break;

        case DemoSpeedUp:
            Serial.println("Change speed while running");
            checked("setSpeedRpm", motor.setSpeedRpm(FASTER_RPM));
            scheduleNext(DemoCoast, 5000);
            break;

        case DemoCoast:
            Serial.println("Natural stop");
            checked("coast", motor.coast());
            scheduleNext(DemoReverse, 3000);
            break;

        case DemoReverse:
            Serial.println("Reverse run");
            checked("runRPM reverse", motor.runRPM(REVERSE_RPM, BLD510x::Reverse));
            scheduleNext(DemoBrake, 5000);
            break;

        case DemoBrake:
            Serial.println("Braking stop");
            checked("emergencyBrake", motor.emergencyBrake());
            scheduleNext(DemoDone, 3000);
            break;

        case DemoDone:
            Serial.println("Demo complete; holding stopped");
            checked("coast", motor.coast());
            scheduleNext(DemoDone, 10000);
            break;
    }
}

bool checkFault()
{
    uint8_t faultBits = 0;

    if (!motor.readFault(faultBits))
    {
        Serial.println("Fault read failed; braking");
        motor.emergencyBrake();
        return true;
    }

    if (faultBits == BLD510x::FaultNone)
    {
        return false;
    }

    Serial.print("Fault bits: 0x");
    Serial.println(faultBits, HEX);
    motor.emergencyBrake();
    return true;
}

void setup()
{
    Serial.begin(PC_BAUD);
    while (!Serial && millis() < 3000)
    {
        delay(10);
    }

    Serial.println();
    Serial.println("BLD-510x RS-485 example");

    motor.begin(MOTOR_BAUD);
    motor.setResponseTimeout(RESPONSE_TIMEOUT_MS);

    // Uncomment this line when you want to see every Modbus TX/RX frame.
    // motor.setDebugStream(&Serial);

    checked("useAddress", motor.useAddress(MOTOR_ADDRESS));
    checked("setPolePairs", motor.setPolePairs(MOTOR_POLE_PAIRS));
    motor.setMaxRPM(7000.0f);

    checked("setAccelerationDeceleration", motor.setAccelerationDeceleration(10, 10));
    checked("setSpeedRpm 0", motor.setSpeedRpm(0));
    checked("coast", motor.coast());

    scheduleNext(DemoStartForward, 2000);
}

void loop()
{
    if (checkFault())
    {
        delay(500);
        return;
    }

    runDemoStep();

    float rpm = 0.0f;

    if (motor.getRPM(rpm))
    {
        Serial.print("Motor RPM: ");
        Serial.println(rpm);
    }

    delay(500);
}
