#include "TurnSensor.h"
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

void setup()
{
    turnSensorSetup();
}

void loop()
{
    buttonA.waitForButton();
    delay(1000);
    motors.setSpeeds(200, -200);

    while (turnAngle < 180) turnSensorUpdate();

    motors.setSpeeds(0,0);
    turnSensorReset();
}