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

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
// servo pwm pin, 5 IS PLACEHOLDER
#define SERVO_PWM_PIN 4

#define ETX '\r'
#define STX 'B'
// Using Netstrings format: length:msg,
// MSG format: "{LEN}:{ADDRESS}{COMMAND}{DATA}{CHECKSUM}{PWM}\r"
char* buf[4];
const char* ssid = "**********";
const char* password = "**********";

WiFiServer server(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

void setup() {
  // Initialize PWM pin, TODO: Verify pin mode correct!
  pinMode(SERVO_PWM_PIN, OUTPUT);
  Serial1.begin(115200);
  WiFi.begin(ssid, password);
//  TODE: write to port instead
  Serial1.print("\nConnecting to "); Serial1.println(ssid);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  
  //start UART and the server
  Serial.begin(115200);
  server.begin();
  server.setNoDelay(true);
// TODO: write to port instead  
  Serial1.print("Ready! Use 'telnet ");
  Serial1.print(WiFi.localIP());
  Serial1.println(" 23' to connect");
}

void parse_cmd(int client)
{
  char input;
  do
  {
    input = serverClients[client].read();  
  }while(serverClients[client].available() && input != 'B');
  int i = 0;
  while(serverClients[client].available() && i++ < 4)
  {
    buf[i] = serverClients[client].read();
  }
  Serial1.print(buf);
  Serial.print(buf);
  if (input = serverClients[client].read() != '\r')
  {
    analogWrite(SERVO_PWM_PIN, input); 
  }
}

void loop() {
  uint8_t i;
  //check if there are any new clients
  if (server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()){
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        Serial1.print("New client: "); Serial1.print(i);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }
  //check clients for data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
      if(serverClients[i].available()){
        //get data from the telnet client and push it to the UART
        parse_cmd(i);
      }
    }
  }
  //check UART for data
  yield()
  }
}
