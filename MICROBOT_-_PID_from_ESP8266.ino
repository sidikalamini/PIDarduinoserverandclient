// MiCROBOT V12.2
// By Julkifli Awang Besar
// PID code for MiCROBOT Hikari with MiCROBOT PCB chassis

//Edited by neo
//CHANGES:
//*** added auto stop function after 1 round is completed
//*** put codes in separate function for readibility
//*** robot max speed value is base speed, to prevent robot from doing fast turn and out of the track

// PLEASE USE THE LATEST QTRSensors LIBRARY VERSION 4.0
// Install from Arduino Library Manager, search for "QTRSensors" and the version is 4.0 https://github.com/pololu/qtr-sensors-arduino
#include <QTRSensors.h>
#include "CytronMotorDriver.h"
#include <CytronWiFiShield.h>
#include <CytronWiFiClient.h>
#include <CytronWiFiServer.h>
#include <SoftwareSerial.h>

#define BUTTON 2
#define REMOTE_BUTTON 2 // Jsumo micro start module
#define BUZZER 8
#define LED 4                                                                                                   
#define NUM_SENSORS 8
#define NUM_OF_SAMPLES_PER_SENSOR 4
#define TIMEOUT 2500
#define EMITTER_PIN 7

// initialize sensor object
QTRSensors qtr;
const char *ssid = "CytronESPShield";
const char *pass = "RobotHeadToToe";
char sensorvalue[50];
String stringvalue = "";
IPAddress destserver(192,168,4,1);

const String data = "GET /KD HTTP/1.1\r\n"
                    "Host: 192.168.4.1\r\n"
                    "Connection: close\r\n\r\n";

CytronMD motor1(PWM_PWM, 11, 3);   // PWM 1A = Pin 11, PWM 1B = Pin 3.
CytronMD motor2(PWM_PWM, 9, 10); // PWM 2A = Pin 9, PWM 2B = Pin 10.

// tuning parameters
float KPi = 0.000028;  
float KDi = 0.000028;//8;
float KIi = 0.000028;
unsigned int base_speed = 0;//110; // base speed @ max speed for robot (range 0 to 255 only)
unsigned int max_speed = 150;//110; // base speed @ max speed for robot (range 0 to 255 only)
// end of tuning

// global variables
int current_error = 0;
int previous_error = 0;
int proportional = 0;
int derivative = 0;
int integral = 0;
int adjustment = 0;
int left_motor_speed = 0; 
int right_motor_speed = 0;
unsigned int center_position = 3700;//((NUM_SENSORS - 1) * 1000) / 2;
unsigned int current_position = 0;
unsigned int sensor_values[NUM_SENSORS];

void setup()
{   
    // QTRsensors configuration
    qtr.setTypeAnalog();
    qtr.setSensorPins((const unsigned char[]) { A0, A1, A2, A3, A4, A5, A6, A7 }, NUM_SENSORS);
    qtr.setSamplesPerSensor(NUM_OF_SAMPLES_PER_SENSOR);
    qtr.setEmitterPin(EMITTER_PIN);
    qtr.setTimeout(TIMEOUT);
    // end QTRSensors configuration
    pinMode(LED, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(BUTTON, INPUT);
    // end pins configuration

  Serial.begin(9600);
  
  if(!wifi.begin(6, 5))
    {
      Serial.println(F("Error talking to shield"));
      while(1);
    }
  Serial.println(wifi.firmwareVersion());
  Serial.print(F("Mode: "));Serial.println(wifi.getMode());// 1- station mode, 2- softap mode, 3- both
  // Uncomment these 2 lines if you are using static IP Address
  // Serial.println(F("Setup wifi config"));
  // wifi.config(ip);
  Serial.println(F("Start wifi connection"));
  if(!wifi.connectAP(ssid, pass))
  {
    Serial.println(F("Error connecting to WiFi"));
    while(1);
  } 
  Serial.print(F("Connected to "));Serial.println(wifi.SSID());
  Serial.println(F("IP address: "));
  Serial.println(wifi.localIP());
  wifi.updateStatus();
  Serial.println(wifi.status()); //2- wifi connected with ip, 3- got connection with servers or clients, 4- disconnect with clients or servers, 5- no wifi
  
  while (!base_speed > 0)
    upload(data);
  

  //start ca;onratopm  
    // double beep
   tone(13, 1000, 100);
    delay(200);
    tone(13, 1000, 100);

    // sensor calibration mode
      digitalWrite(4, HIGH);
      for (uint16_t i = 0; i < 400; i++)
      {
        qtr.calibrate();
      }
     digitalWrite(4, LOW);
    // end sensor calibration
    tone(13, 1000, 100);
    delay(200);
    delay(1000);
}
    // LED blinking fast, READY mode
    // press button again to move the robot
    //while (!digitalRead(BUTTON)) {    
//        digitalWrite(LED, HIGH);
//        delay(150);
//        digitalWrite(LED, LOW);
//        delay(150);
//    }
//    digitalWrite(LED, LOW);

//start main loop
void loop() 
{
    
    // check for robot position
    read_sensor();    
    // check for finish line
    //start_finish_check();
    // run pid controller function
    run_pid();
}
//end main loop

//define function
void read_sensor(void)
{
    current_position = qtr.readLineBlack(sensor_values);    // if the line is white, use qtr.readLineWhite(sensors_values)
}

void run_pid(void)
{
    // calculate the error
    current_error = current_position - center_position;
    //Serial.println("Current Error: "+String(current_error));
    // PID calculation
    proportional = current_error;
    
    derivative = current_error - previous_error;
    integral = current_error + previous_error;
    
    adjustment = (KPi/2000 * proportional)+ (KDi/2000* derivative)+(KIi/10000*integral);
    //Serial.println("Adjustment :"+String(adjustment));
    // set up motor base speed + adjustment
    left_motor_speed = base_speed - adjustment; 
    right_motor_speed = base_speed + adjustment;

    // constrain speed value from 0 to base speed
    left_motor_speed = constrain(left_motor_speed, 0, max_speed); 
    right_motor_speed = constrain(right_motor_speed, 0, max_speed);

    // store the error
    previous_error = current_error;

    run_motor(left_motor_speed, right_motor_speed);
}

void run_motor(int l_speed, int r_speed)
{
    
    //Serial.print("L: "+String(l_speed)+" R: "+String(r_speed)+"\n");
    //delay(1000);
    
    // send output to left motor
    if(l_speed > 70) //maximize speed if
    {
    l_speed = 90;
    r_speed = 0;
    delay(10);
      }
    
    if (r_speed >70){
     r_speed=90;
     l_speed=0;
    delay(10);
    }
    
    motor1.setSpeed(l_speed);   // Motor 1 runs forward at 50% speed.
    motor2.setSpeed(r_speed);
    
}

void start_finish_check(void) {
    static int counter = 0;
    static unsigned long start_timer_ms = 0;
    const int sensor_threshold = 500;
    const int one_second = 1000;    // 1 sec = 1000ms

    // reached finish line
    if (counter == 2) {
        while (true) {
            //turn off edf
            //ESC.write(1000);
            // stop the motor
            run_motor(0, 0);           
        }
    }

    // right and left most sensors detects black line
    if (sensor_values[0] > sensor_threshold && sensor_values[7] > sensor_threshold) {
        if (counter == 0) {
            start_timer_ms = millis();
            counter = counter + 1;
        }
        else if (counter > 0 && (millis() - start_timer_ms) > one_second) {
            counter = counter + 1;
        }
    }    
}

void upload(String data)
{
  const char* host = "192.168.4.1";
  ESP8266Client client;
  if (!client.connect(host, 80))
  {
    Serial.println(F("Failed to connect to server."));
    client.stop();
    return;
  }
  
  //const char *httpRequest = "GET /KD HTTP/1.1\r\n"
  //                         "Host: 192.168.4.1\r\n"
  //                         "Connection: close\r\n\r\n";
  if(!client.print(data))
  {
    Serial.println(F("Sending failed"));
    client.stop();
    return;;
  }

  // set timeout approximately 5s for server reply
  int i=5000;
  while (client.available()<=0&&i--)
  {
    delay(1);
    if(i==1) {
      Serial.println(F("Timeout"));
      return;
      }
  }

  while (client.available()>0)
  {
  
    //char sensorvalue = (char)client.read();
    stringvalue= client.readStringUntil('\n');
    //stringvalue += String((char)client.read());
    //Serial.write(client.read());

  }
  client.stop();
  
  //Serial.print(svf);
  //Serial.print("\n");
  delay(1000); 
  if (stringvalue.indexOf("IPD")>0)
  {
  int cindex = stringvalue.lastIndexOf(":");     
  String svf = stringvalue.substring(cindex+1);
  int column= svf.lastIndexOf("#");
  int firstcomma = svf.indexOf(",");
  int lastcomma = svf.lastIndexOf(",");
  String KP = svf.substring(0,firstcomma);
  String KD = svf.substring(firstcomma+1,lastcomma);
  String KI = svf.substring(lastcomma+1,column);
  String BASE = svf.substring(column+1);
  
  base_speed = BASE.toInt();
  KPi = KP.toInt();
  KDi = KD.toInt();
  KIi = KI.toInt();
  
  
  
  //Serial.print("KP :"+KP);
  //Serial.print(" KD :"+KD);
  //Serial.print(" KI :"+KI);
  //Serial.print(" BASE :"+BASE);
  
  //Serial.println("KPi:"+String(KPi));    
  //Serial.println("KDi:"+String(KDi));
  //Serial.println("KIi:"+String(KIi));
  //Serial.println("base_speed:"+String(base_speed));  
  stringvalue = "";
 
  }
  //Serial.print("\n");
  return KPi;
  return KDi;
  return KIi;
  return base_speed;
 
} 
