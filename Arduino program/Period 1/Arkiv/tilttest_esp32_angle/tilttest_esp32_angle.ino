#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;   // create MPU6050 object to read accel/gyro
int16_t ax, ay, az;  // accelerometer
int16_t gx, gy, gz;  // gyroscope


#define potpin 0  // analog pin used to connect the potentiometer
const int pwmPin = 10;  // GPIO pin for PWM output
const int switchPin = 4;
const int potPin = 0;
const int freq = 50;   // PWM frequency in Hz (50Hz for servo applications) 50hz (20000)
const int resolution = 14; // 14-bit resolution
const uint8_t SDA_pin = 8;
const uint8_t SCL_pin = 9;
double Kp = 1; //Proportial part of pid controller
double ref = 0.1; //Refrence value
double y = 0; //output signal (vinkelacceleration)
double e = 0; //error
double Ki = 0.00;//01; //contribution from integrating part of controller
double I = 0;
int pw = 0; //Pulse width
double DC = 0;

unsigned long timer1 = millis();
unsigned long last_update = 0;

void calibration() {
  // calibration - Needs to be flat and warmed up
  Serial.println("Current Offsets: ");
  accelgyro.PrintActiveOffsets();
  Serial.print("Calibrating accelerometer ");
  accelgyro.CalibrateAccel(6);
  Serial.print("\nCalibrating gyroscope ");
  accelgyro.CalibrateGyro(6);
  Serial.println("\nOffsets:");
  accelgyro.PrintActiveOffsets();
}

void printCool() {
  if (millis() > timer1 + 500) {
    timer1 = millis();
    // display tab-separated accel/gyro x/y/z values
    Serial.println("a/g:\tXAccel\tYAccel\tZAccel\tXGyro\tYGyro\tZGyro");
    Serial.print("a/g:\t");
    Serial.print((double)ax / 16384, 2);
    Serial.print("\t");
    Serial.print((double)ay / 16384, 2);
    Serial.print("\t");
    Serial.print((double)az / 16384, 2);
    Serial.print("\t");
    Serial.print((double)gx / 16384, 2);
    Serial.print("\t");
    Serial.print((double)gy / 16384, 2);
    Serial.print("\t");
    Serial.println((double)gz / 16384, 2);

    // print potentiometer
    Serial.print("e = ");
    Serial.println(e);
    Serial.print("y = ");
    Serial.println(y);
    Serial.print("pw = ");
    Serial.println(pw);
  }
}

void escSetup()
{
  ledcWrite(0, (2400 * 16384) / 20000); //Set PWM Duty cycle which is limited to 0-16384
  delay(2000);
  ledcWrite(0, (1000 * 16384) / 20000); //Set PWM Duty cycle which is limited to 0-16384
  Serial.println("ESC set up complete");
  delay(1000);
}

void plotCool()
{
  if (millis() > timer1 + 100) {
    timer1 = millis();
    Serial.print("pw:");
    Serial.print(pw);
    Serial.print(",");
    Serial.print("gx:");
    Serial.print((double)gx/16384,2);
    Serial.print(",");
    Serial.print("e:");
    Serial.print(e,2);
    Serial.print(",");
    Serial.print("P:");
    Serial.print(Kp*e,2);
    Serial.print(",");
    Serial.print("I:");
    Serial.print(I);
    Serial.print(",");
    Serial.print("e*1000:");
    Serial.println((e*1000));
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  Wire.begin();         // join I2C bus (I2Cdev library doesn't do this automatically)
  pinMode(switchPin, INPUT);   
  Serial.begin(38400);  // initialize serial communication
  Serial.println("");
  // Configure PWM settings
  if (ledcAttach(10, 50, 14))//ledcAttach(pwmPin, freq, resolution
  {
    Serial.println("PWM setup sucsessful");  //  50 Hz, 16-bit resolution  (LEDControl är en pwm generator i esp32)
  }
  else
  {
    Serial.println("PWM setup failed");
  }
  delay(2000);

  // initialize MPU6050
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5); //MPU6050_DLPF_BW_5 <- BW=5 //Filter

  // verify connection MPU6050
  Serial.print("Testing device connections... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful!" : "MPU6050 connection failed...");
  // calibration();   // calibration MPU6050 - Needs to be flat and warmed up
  //escSetup();
  delay(3000);
}

void loop() {
  //Serial.print("sw:");
  //Serial.println(digitalRead(switchPin));
  //Serial.print("pot:");
  //Serial.println((analogRead(potPin)/4095.0)*3.3);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // read raw accel/gyro measurements from device

  if(digitalRead(switchPin))
  {
    if(last_update != millis())
    {
      last_update = millis(); //This means the controller will update only once every 1000ms.
      //plotCool();
      //printCool(); // display tab-separated accel/gyro x/y/z values
      y = atan((double)ay / (double)az);
      Serial.print("y:");
      Serial.print(y);
      if(abs(y)<0.01){
        y = 0;
      }
      e = ref - y;
      I = I + Ki*e;
      I = max(min(I, 0.5), -0.5);
      pw = 2400*(Kp * e + I); //Parantesen går mellan 0 och 1; 
      Serial.print(",");
      Serial.print("pw:");
      Serial.println((double)pw/1000);
    }
    
  }
  else
  {
    pw = map(analogRead(potPin), 0, 4095.0, 900, 2400);
    Serial.println("pw");
    Serial.println(pw);
  }
  pw = max(min(pw, 2400), 1400); // pulse width limited to (1000,1600) for easier testing
  int dutyCycle = (pw * 16384) / 20000;  // Convert microseconds to duty cycle when resolution is 14 bit (16384) and frequency 50hz (20000)
  //Serial.println("DC:");
  //Serial.println(dutyCycle);
  ledcWrite(pwmPin, dutyCycle); //Set PWM Duty cycle which is limited to 0-16384
  //delay(1);
}
