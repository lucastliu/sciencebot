
/**
 * Packets are of the form "z<port> <power>\n".
 * port: integer 1..4
 * power: float -1..1
 */

#include <AFMotor.h>

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
AF_DCMotor motors[4] = {motor1, motor2, motor3, motor4};

/**
 * Sets the percentage of power and direction of the specified motor.
 * @param motor the motor whose power should be set
 * @param power -1..1 the power and direction to set the motor to
 */
void setPower(int port, float power) {
  motors[port - 1].run(power >= 0 ? FORWARD : BACKWARD);
  motors[port - 1].setSpeed(abs(power * 255));
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20);
}

void loop() {
  if (Serial.available() && Serial.read() == 'z') {
    int port = Serial.parseInt(SKIP_WHITESPACE);
    float power = Serial.parseFloat(SKIP_WHITESPACE);
    setPower(port, power);
  }
}
