

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "math.h"

//WIFI STUFF***********************************************************************************************************************************************************

//Wifi stuff
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>

//Wifi definitions
#define ESPNOW_WIFI_CHANNEL 6
bool data_avaliable = false;
uint16_t received_data;

//Wifi class
// Creating a new class that inherits from the ESP_NOW_Peer class is required.
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) //Received a 16 bit message
  {
  received_data = *((uint16_t *)data);  // Cast data to uint16_t and dereference
  data_avaliable = true;
  }
};

std::vector<ESP_NOW_Peer_Class> masters;  // List of all the masters. It will be populated when a new master is registered

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
      Serial.println("Failed to register the new master");
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}

//**********************************************************************************************************************************************************************************************
MPU6050 accelgyro;   // create MPU6050 object to read accel/gyro
int16_t ax, ay, az;  // accelerometer
int16_t gx, gy, gz;  // gyroscope

//pin definitions
const int yrefPot = 0;      //analog pin used to connect the potentiometer
const int xrefPot = 1;      // analog pin used to connect the potentiometer
const int liftPot = 2;      // analog pin used to connect the potentiometer
const int switchPin = 3;
const int M1 = 4;           //PWM output Motor 1
const int M2 = 5;           //PWM output Motor 2
const int M3 = 6;           //PWM output Motor 3
const int M4 = 7;           //PWM output Motor 4
const int SDA_pin = 9;      //Kan ändras, detta är bara default. Pin 8 används även av LED.
const int SCL_pin = 8;
bool force_zero = false;
unsigned long last_update = 0;
unsigned long timer1 = millis();
unsigned long counter = 0;
unsigned long last_transmission = 0;
//PWM settings
const int freq = 250;        // PWM frequency in Hz (50Hz for servo applications) 50hz (20000)
const int resolution = 14;  // 14-bit PWM resolution
//Loop time (ms)
double loopTime = 1000000/freq; //Loop time in micros
//Pid coefficients
double Kp = 800;                 //Proportial part of pid controller
double Ki =3250/freq;                //contribution from integrating part of controller
double Kd = 0*freq;            //Denna är så stor så att vi inte måste dividera med tiden (1ms) för varje iteration av loopen
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
//Motor power variables (determined by 900-2400us pulses)
int pw = 0;                       //Pulse width
int P1 = 0;
int P2 = 0;
int P3 = 0;
int P4 = 0;
int PCOM = 0; 
int PDIFF_x = 0;
int PDIFF_y = 0;


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
  if (millis() + 500 >= timer1) {
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
    /*
    Serial.print("x_ref*100:");
    Serial.print(ref_x*100);
    Serial.print(",");
    Serial.print("y_ref*100:");
    Serial.print(ref_y*100);
    Serial.print(",");
    Serial.print("gx:");
    Serial.println(gx);*/
    Serial.print("ref_x*10:");
    Serial.print(ref_x*10);
    Serial.print(",");
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
    int dutyCycle = (pulseWidth*16384) / loopTime;  // Convert microseconds to duty cycle when resolution is 14 bit (16384) and frequency 250hz (4000us)
    ledcWrite(pin, dutyCycle); //Set PWM Duty cycle which is limited to 0-16384
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, NULL);

  Serial.println("Wifi setup complete");

  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  setCpuFrequencyMhz(240);
  Wire.begin(SDA_pin, SCL_pin);         
  Wire.setClock(400000);
  pinMode(switchPin, INPUT);   
  Serial.begin(115200);  // initialize serial communication
  Serial.println("");
  // Configure PWM settings
  if (ledcAttach(M3, freq, resolution) && ledcAttach(M4, freq, resolution) && ledcAttach(M1, freq, resolution) && ledcAttach(M2, freq, resolution))//ledcAttach(pwmPin, freq, resolution
  {
    Serial.println("PWM setup sucsessful");  //  50 Hz, 14-bit resolution  (LEDControl är en pwm generator i esp32)
  }
  else
  {
    Serial.println("PWM setup failed");
    delay(20000);
  }
  escSetup();
  
  // initialize MPU6050
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  //accelgyro.setDLPFMode(0x05); // BandWidth: 6=5Hz, 5=10Hz, 4=20Hz, ..., 0=256Hz
  accelgyro.setFullScaleGyroRange(0x03);
  
  // verify connection MPU6050
  Serial.print("Testing device connections... ");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful!" : "MPU6050 connection failed...");
  delay(1000);
  calibration();   // calibration MPU6050 - Needs to be flat and warmed up
  delay(3000);
}

void loop() 
{
  if (data_avaliable) 
  {
    last_transmission = millis();
    uint8_t commandByte = (received_data >> 8) & 0b11111111;  // Higher 8 bits
    uint8_t valueByte = received_data & 0b11111111;         // Lower 8 bits
    if(commandByte == 0b00000001)
    {
       PCOM = map(valueByte, 0, 255, 900, 2200);
       //Serial.println(PCOM);
    }
    else if(commandByte == 0b00000010)
    { 
       ref_x = valueByte-128;
    }
    else if(commandByte == 0b00000011)
    {
      ref_y = valueByte-128;
    }
    else if(commandByte == 0b00000100)
    {
      force_zero = valueByte & 0b00000001; //We only check LSB of valueByte
    }
    //Serial.println(received_data,BIN);
    data_avaliable = false;
  }
  else if (last_transmission + 1000 < millis()) //If we have not received a message for a second we reboot for safety
  {
    ESP.restart();
  }
  if(last_update + loopTime <= micros()) //Vi uppdaterar varannan ms
  {
    //Serial.println(millis()); //Med denna kan man kolla hur lång tid en cykel tar
    last_update = micros(); //Egentligen tar en cykel typ 1500us.
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // read raw accel/gyro measurements from device 

    if(force_zero) //Force rotation refrence to 0
    {
      ref_y = 0.00;    
      ref_x = 0.00;    
    }

    else //Refrence set with potentiometers
    {
      ref_x = 0;// Detta implementeras senare
      ref_y = 0; //
    }
    //printCool();
    //plotCool();

    //PID
    y_x = (double)gx/16384; //0 to 1 where 1 is 1000degrees/s
    if(abs(y_x)<0.01) //Ta bort jätte små y för att förhindra drift
    { 
      y_x = 0;
    }
    e_x = ref_x - y_x;
    I_x = I_x + Ki*e_x;
    D_x = (e_x-e_x_prev);
    e_x_prev = e_x;
    PDIFF_x = Kp * e_x + I_x + Kd*D_x; 

    y_y = (double)gy/16384; //0 to 1 where 1 is 1000degrees/s
    if(abs(y_y)<0.01)
    { 
      y_y = 0;
    }
    e_y = ref_y - y_y;
    I_y = I_y + Ki*e_y;
    D_y = (e_y-e_y_prev);
    e_y_prev = e_y;
    PDIFF_y = Kp * e_y + I_y + Kd*D_y;   

    //Calculate power (pulse width) for the different motors. 
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
