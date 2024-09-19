#include <Servo.h>

Servo esc;  // Create a Servo object for the ESC

void setup() {
  esc.attach(6);  // Attach the ESC to Pin 6
  esc.writeMicroseconds(1000);  // Set throttle to minimum (1000 µs)
  delay(2000);  // Wait 2 seconds for ESC to initialize
}

void throttle(float i) //Throttle speed 0 to 1
{
    esc.writeMicroseconds(1000 + 100*i);  // Set throttle to 10%
}

void loop() {
  throttle(.1);
  delay(1000);  // Hold for 1 second

  throttle(.2);
  delay(1000);  // Hold for 1 second

  esc.writeMicroseconds(1000);  // Stop the motor
  delay(1000);
}
