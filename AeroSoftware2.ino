#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#define PIN D3
#define pwmSDA D2
#define pwmSCL D1

#define MIN_PULSE_WIDTH       450
#define MAX_PULSE_WIDTH       2450
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             200

//Configure the i2c addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x52);
//not sure what these are doing, not i2c addresses, the i2c address is contained
//in the header file. 
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

String data;
static int servo_position  = 30;
static int servo_direction = 0;
static int counter = 0;
static int turn = 0;
static int PreviousLightRequest = 0;
bool motor_enable = 0;

void setup() 
{ 
  //Setup the Motor Enable PIN
  //https://iotbytes.wordpress.com/nodemcu-pinout/
  pinMode(D0, INPUT);      //Switch is connected to this
  //pinMode(2, OUTPUT);     // ESP onboard LED

  //Setup the i2c bus
  Wire.begin(pwmSDA, pwmSCL);
  pwm.begin();
  pwm.setPWMFreq(200);

  //Drive motor A
  pinMode(D6, OUTPUT);
  pinMode(D5, OUTPUT);
  digitalWrite(D6, LOW);
  digitalWrite(D5, LOW);
  pwm.setPWM(8, 0, 100);

  //Drive Motor B
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);
    
  //Start the serial port
  Serial.begin(115200);
  //short delay to get things kicked off
  delay(100);

  /* Initialise the sensors */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }

}

void loop() 
{
    //Structure of data definition in AdafruitSensors.h
    sensors_event_t event;

    //Display the results (acceleration is measured in m/s^2) */
    //Pass in the data structure and load with acceleration data.
    //event.acceleration.x  , y, z are the values. 
    accel.getEvent(&event);

    Serial.println(event.acceleration.x);
    
    //check the toggle switch state
    motor_enable = digitalRead(D0);
    if(motor_enable == 1)
    {
       digitalWrite(D6, LOW);
       digitalWrite(D5, HIGH);
       pwm.setPWM(8, 0, 2000);

       digitalWrite(D8, LOW);
       digitalWrite(D7, HIGH);
       pwm.setPWM(9, 0, 2000);
       Serial.println("Motors are enabled");
       
    }
    else
    {
       //TURN THE DRIVERS OFF
       digitalWrite(D6, LOW);
       digitalWrite(D5, LOW);
       pwm.setPWM(8, 0, 4096);

       digitalWrite(D8, LOW);
       digitalWrite(D7, LOW);
       pwm.setPWM(9, 0, 4096);

       Serial.println("Motors are disabled");
    }
    Serial.println(motor_enable);
    delay(1000);
}

