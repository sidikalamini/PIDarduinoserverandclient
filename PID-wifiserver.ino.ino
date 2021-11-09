+-/*
  WiFi Web Server LED Blink

 A simple web server that lets you blink an LED via the web.
 This sketch will print the IP address of your WiFi Shield (once connected)
 to the Serial monitor. From there, you can open that address in a web browser
 to turn on and off the LED on pin 13.

 If the IP address of your shield is yourAddress:
 http://yourAddress/H turns the LED on
 http://yourAddress/L turns it off

 Circuit:
 * WiFi shield attached
 * LED attached to pin 13
 created 25 Nov 2012
 by Tom Igoe
 modified 16 Sept 2015
 by Ng Beng Chet
 */

/* This example utilises softAP ability of the shield.
 *  
 * You can configure softAP settings with function below
 * WiFi.softAP(ssid, password, channel id, encryption)
 * 
 * by default channel id = 1, encryption = WPA_WPA2_PSK
 * 
 * Encryption option:
 * OPEN - 0
 * WPA_PSK - 2
 * WPA2_PSK - 3
 * WPA_WPA2_PSK - 4
 * 
 */
 
#include <CytronWiFiShield.h>
#include <CytronWiFiServer.h>
#include <SoftwareSerial.h>
#define WiFi wifi
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

const char ssid[] = "CytronESPShield";      //  your network SSID (name)
const char pass[] = "RobotHeadToToe";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
String sensorreading ="takde sensor";

char buffer1 [18]; // a few bytes larger than your LCD line
char buffer2 [18]; // a few bytes larger than your LCD line
char buffer3 [18]; // a few bytes larger than your LCD line
char buffer4 [18]; // a few bytes larger than your LCD line
const int analogInPin = A0;
const int analogInPin2 = A1;
const int analogInPin3 = A2;
const int analogInPin4 = A3;
int sensorValue = 0; 
int sensorValue2 = 0; 
int sensorValue3 = 0;
int sensorValue4 = 0;
int outputValue = 0; 
int outputValue2 = 0; 
int outputValue3 = 0;
int outputValue4 = 0;
int oldValue = 0;
int oldValue2 = 0;
int oldValue3 = 0;
int oldValue4 = 0;

ESP8266Server server(80);
bool status = false;

void setup() {
  Serial.begin(115200);      // initialize serial communication
  pinMode(13, OUTPUT);      // set the LED pin mode

  lcd.begin(16, 2);
    // Print a message to the LCD.
  lcd.setRGB(100,100,100);

  // check for the presence of the shield:
  if (!WiFi.begin(2, 3)) {
    Serial.println("WiFi shield not present");
    while (true);       // don't continue
  }

  String fv = WiFi.firmwareVersion();
  Serial.println(fv);

  // attempt to connect to Wifi network:
  WiFi.setMode(WIFI_AP);
  if(!WiFi.softAP(ssid, pass))
  //if(!WiFi.softAP(ssid, pass, 5, WPA_PSK)) // use WPA encryption
    Serial.println("Setting softAP failed");
  Serial.println(WiFi.softAPIP());
  server.begin();                           // start the web server on port 80
}


void loop() {
  sensorValue = analogRead(analogInPin);
  sensorValue2 = analogRead(analogInPin2);
  sensorValue3 = analogRead(analogInPin3);
  sensorValue4 = analogRead(analogInPin4);
  outputValue = map(sensorValue, 1023, 0, 0, 255);
  outputValue2 = map(sensorValue2, 1023, 0, 0, 255);
  outputValue3 = map(sensorValue3, 1023, 0, 0, 255);
  outputValue4 = map(sensorValue4, 1023, 0, 0, 255);
  //sprintf (outputValue, "I = %3u",outputValue);
  //sprintf (outputValue2, "I = %3u",outputValue2);
  //sprintf (outputValue3, "I = %3u",outputValue3);
   String combinedValue=String(outputValue)+","+String(outputValue2)+","+String(outputValue3)+"#"+String(outputValue4);
   if (oldValue !=outputValue) 
    {
     lcd.setCursor(0,0);
     
     sprintf (buffer1, "%3u", outputValue);
     lcd.print(buffer1);  

    }
    
       // lcd.print(dtaUart);
    
    oldValue = outputValue;

     if (oldValue2 !=outputValue) 
    {
     lcd.setCursor(8,0);
     
     sprintf (buffer2, "%3u", outputValue2);
     lcd.print(buffer2);
     

    }
    
       // lcd.print(dtaUart);
    
    oldValue2 = outputValue2;

     if (oldValue3 !=outputValue3) 
    {
     lcd.setCursor(0,1);
     
     sprintf (buffer3, "%3u", outputValue3);
     lcd.print(buffer3);
     

    }
    
       // lcd.print(dtaUart);
    
    oldValue3 = outputValue3;
     
     if (oldValue4 != outputValue4) 
    {
     lcd.setCursor(8,1);
     sprintf (buffer4, "%3u", outputValue4);
     lcd.print(buffer4);
     

    }
    
       // lcd.print(dtaUart);
    
    oldValue4 = outputValue4;
  
 // lcd.setCursor(0,0);
  //lcd.print(outputValue);
  //lcd.setCursor(8,0);
  //lcd.print(outputValue2);
  //lcd.setCursor(0,1);
  //lcd.print(outputValue3);
  //lcd.setCursor(8,1);
  //lcd.print(outputValue4);
  
  ESP8266Client client = server.available();   // listen for incoming clients

  if(!client) return;
  Serial.println("new client");
  
  if (client.connected()) //if client is present and connected
  {             
      String s = client.readStringUntil('\r');   //get the first line of request       
      // Check to see if the client request was "GET /H" or "GET /L":
      if (strstr(s.c_str(),"GET /KD")) 
        sensorreading =  combinedValue;              // GET /H turns the LED onq
      else 
        sensorreading ="Salah alamat";   
      
      while(client.available())      
        Serial.write(client.read());                    // print the client request out the serial monitor
        
      // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
      // and a content-type so the client knows what's coming, then a blank line:
      //client.println("HTTP/1.1 200 OK");
      //client.println("Content-type:text/html");
      //client.println();

      // the content of the HTTP response follows the header:
      client.print(sensorreading);
      
      // The HTTP response ends with another blank line:
      //client.println();
      // close the connection:
      client.stop();
      Serial.println(sensorreading);
      Serial.println("client disonnected");
  }
}
