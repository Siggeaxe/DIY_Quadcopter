#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "math.h"

MPU6050 accelgyro;   // create MPU6050 object to read accel/gyro
int16_t ax, ay, az;  // accelerometer
int16_t gx, gy, gz;  // gyroscope

//pin definitions
const int tiltPot = 0;      // analog pin used to connect the potentiometer
const int liftPot = 1;      // analog pin used to connect the potentiometer
const int M1 = 4;           //PWM output Motor 1
const int M2 = 5;           //PWM output Motor 2
const int M3 = 6;           //PWM output Motor 3
const int M4 = 7;           //PWM output Motor 4
const int switchPin = 2;
const int SDA_pin = 8;      //Kan ändras, detta är bara default. Pin 8 används även av LED.
const int SCL_pin = 9;
//PWM settings
const int freq = 50;        // PWM frequency in Hz (50Hz for servo applications) 50hz (20000)
const int resolution = 14;  // 14-bit PWM resolution
//Pid coefficients
double Kp = 0.8*70;                 //Proportial part of pid controller
double Ki = 0.8*0.3;                //contribution from integrating part of controller
double Kd = 0.1/0.001;            //Denna är så stor så att vi inte måste dividera med tiden (1ms) för varje iteration av loopen
//Pid variables
double ref_x = 0.00;              //Refrence value horizontal x axis
double y_x = 0;                   //output signal (vinkelacceleration)
double e_x = 0;                   //error
double e_x_prev = 0;
double I_x = 0;
double D_x = 0;

double ref_y = 0.00;              //Refrence value horizontal y axis
double y_y = 0;                   //output signal (vinkelacceleration)
double e_y = 0;                   //error
double e_y_prev = 0;
double I_y = 0;
double D_y = 0;
//Motor power (determined by 900-2400us pulses)
int pw = 0;                //Pulse width
int P1 = 0;
int P2 = 0;
int P3 = 0;
int P4 = 0;
int PCOM = 0; 
int PDIFF_x = 0;
int PDIFF_y = 0;
unsigned long last_update = 0;
unsigned long timer1 = millis();
unsigned long counter = 0;

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
    Serial.println(e_x);
    Serial.print("y = ");
    Serial.println(y_x);
    Serial.print("pw = ");
    Serial.println(pw);
  }
}

void escSetup() //Oklart om denna funktion behövs
{
  Serial.println("ESC Setup begin \n high");
  setPulseWidth(M1,2400);
  setPulseWidth(M2,2400);
  setPulseWidth(M3,2400);
  setPulseWidth(M4,2400);
  delay(10000);
  Serial.println("low");
  setPulseWidth(M1,900);
  setPulseWidth(M2,900);
  setPulseWidth(M3,900);
  setPulseWidth(M4,900);
  delay(3000);
  Serial.println("ESC setup complete");
}

void plotCool()
{
  if (millis() > timer1 + 50) {
    timer1 = millis();
    Serial.print("PDIFF_x:");
    Serial.print(PDIFF_x);
    Serial.print(",");
    Serial.print("e*1000:");
    Serial.print(e_x*100,2);
    Serial.print(",");
    Serial.print("P:");
    Serial.print(Kp*e_x,2);
    Serial.print(",");
    Serial.print("I:");
    Serial.print(I_x);
    Serial.print(",");
    Serial.print("D:");
    Serial.println(Kd*D_x);
  }
}

void setPulseWidth(int pin, int pulseWidth)
{
    int dutyCycle = (pulseWidth * 16384) / 20000;  // Convert microseconds to duty cycle when resolution is 14 bit (16384) and frequency 50hz (20000us)
    ledcWrite(pin, dutyCycle); //Set PWM Duty cycle which is limited to 0-16384
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  setCpuFrequencyMhz(240);
  Wire.begin();         // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.setClock(400000);
  pinMode(switchPin, INPUT);   
  Serial.begin(115200);  // initialize serial communication
  Serial.println("");
  // Configure PWM settings
  if (ledcAttach(M1, freq, resolution) && ledcAttach(M2, freq, resolution) && ledcAttach(M3, freq, resolution) && ledcAttach(M4, freq, resolution))//ledcAttach(pwmPin, freq, resolution
  {
    Serial.println("PWM setup sucsessful");  //  50 Hz, 14-bit resolution  (LEDControl är en pwm generator i esp32)
  }
  else
  {
    Serial.println("PWM setup failed");
  }
  delay(1000);

  // initialize MPU6050
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  accelgyro.setDLPFMode(0x05); // BandWidth: 6=5Hz, 5=10Hz, 4=20Hz, ..., 0=256Hz
  
  // verify connection MPU6050
  Serial.print("Testing device connections... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful!" : "MPU6050 connection failed...");
  delay(1000);
  calibration();   // calibration MPU6050 - Needs to be flat and warmed up
  delay(3000);
  //escSetup();
}

void loop() {
  if(last_update <= millis() + 2 ) //Vi uppdaterar varannan ms
  {
    last_update = millis(); //Egentligen tar en cykel typ 1500us.
    //Serial.println(micros()); //Med denna kan man kolla hur lång tid en cykel tar
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // read raw accel/gyro measurements from device
    PCOM = map(analogRead(liftPot), 0, 4095.0, 900, 2400);
    PCOM = min(max(PCOM, 900), 2200); // Limit maximum lift power for safety
    //Serial.println("PCOM ");
    //Serial.println(PCOM);
    

    if(digitalRead(switchPin)) //PID control
    {
      //Serial.println("PID control");
      plotCool();
      y_x = (double)gx/16384;
      if(abs(y_x)<0.01){ //Ta bort jätte små y för att förhindra drift
        y_x = 0;
      }
      e_x = ref_x - y_x;
      I_x = I_x + Ki*e_x;
      D_x = (e_x-e_x_prev);
      e_x_prev = e_x;
      PDIFF_x = Kp * e_x + I_x + Kd*D_x; 
      //PDIFF_x = max(min(PDIFF_x, 200), -200); // pulse width difference limited to (200,0) for stability

      y_y = (double)gy/16384;
      if(abs(y_y)<0.01){ 
        y_y = 0;
      }
      e_y = ref_y - y_y;
      I_y = I_y + Ki*e_y;
      D_y = (e_y-e_y_prev);
      e_y_prev = e_y;
      PDIFF_y = Kp * e_y + I_y + Kd*D_y; 
      //PDIFF_y = max(min(PDIFF_y, 200), -200); // pulse width difference limited to (200,0) for stability
    }
      
    else //Manual control
    {
      delay(100);
      Serial.println("Manual control");
      PDIFF_x = map(analogRead(tiltPot), 0, 4095.0, -200, 200);
      Serial.print("PDFF: ");
      Serial.println(PDIFF_x);
    }
    //Calculate power (pulse width) for the different motors. 
    //With this calculation we will be able to tilt in one horisontal azis.
    P1 = PCOM + PDIFF_x + PDIFF_y;
    P2 = PCOM + PDIFF_x - PDIFF_y;
    P3 = PCOM - PDIFF_x + PDIFF_y;
    P4 = PCOM - PDIFF_x - PDIFF_y;
    setPulseWidth(M1,P1);
    setPulseWidth(M2,P2);
    setPulseWidth(M3,P3);
    setPulseWidth(M4,P4);
  }
}
