/* 
  WiFiTelnetToSerial - Example Transparent UART to Telnet Server for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP8266WiFi library for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
// servo pwm pin, 5 IS PLACEHOLDER
#define SERVO_PWM_PIN 4

#define ETX '\r'
#define STX 'B'
// Using Netstrings format: length:msg,
// MSG format: "{LIN}:{ANG}:{SERVO_ANG}"
const char* hostname = "Minibot_ESP";
const char* ssid = "**********";
const char* password = "**********";

WiFiUDP port;
char packetBuffer[255];
unsigned int localPort = 9999;

void setup() {
  // Initialize PWM pin, TODO: Verify pin mode correct!
  pinMode(SERVO_PWM_PIN, OUTPUT);

  // Debug Serial
  Serial1.begin(115200);

  WiFi.hostnmame(hostname);
  WiFi.begin(ssid, password);
  
  Serial1.print("\nConnecting to "); Serial1.println(ssid);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  // open UDP port
  port.begin(localPort);
  //start main Serial
  Serial.begin(115200);
  // Debug Serial
  Serial1.print("Ready!  IP: ");
  Serial1.print(WiFi.localIP());
  Serial1.print(" Port: ");
  Serial1.println(localPort);
}

void parse_cmd(int len)
{
  int i = 0;
  int j = 0;
  char val[32];
  char current;
  float lin, ang, servo_ang;

  // parse linear
  while (packetBuffer[i] != ':'){
    val[j] = packetBuffer[i];
    j++;
    i++;
  }
  val[j] = '\0';
  sscanf(val, "%f", &lin);
  i++;
  val[i] = '\0';
  
  // parse angular
  j = 0;
  while (packetBuffer[i] != ':'){
    val[j] = packetBuffer[i];
    j++;
    i++;
  }
  val[j] = '\0';
  sscanf(val, "%f", &ang);
  i++;
  val[i] = '\0';

  // parse servo angle
  for(i; i<len; i++){
    val[j] = packetBuffer[i];
    j++
  }
  val[j] = '\0'
  sscanf(val, "%f", &servo_ang
  
  // clear buffer
  packetBuffer[0] = '\0';
  
  // Debug output
  Serial1.println("Command parsed!");
  // Call PID/Servo Control Here
  
}

void loop() {
  int packetSize = port.parsePacket();

  if (packetSize) {
    int len = port.read(packetBuffer, 255);
    // Sets ending flag in buffer
    if (len > 0) packetBuffer[len-1] = '\0';
    // Debug
    Serial1.println(packetBuffer);
    // Acknowledge
    port.beginPacket(port.remoteIP(), port.remotePort());
    port.write("Ack: ");
    port.write(packetBuffer);
    parse_cmd(packetBuffer, len);
  }
  delay(25);
}
