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
#include <PID_v1.h>

// servo pwm pin, 4 IS PLACEHOLDER
#define SERVO_PWM_PIN 4

#define ETX '\r'
#define STX 'B'

//Robot Physical Paramaters
#define wheel_radius 2.0
#define robot_width  1.0

//Output limits for PID. The second motor is set to proper range before sending to motor driver(see function send_to_motor_driver() for details)
#define min_out 1.0
#define max_out 128.0

// Using Netstrings format: length:msg,
// MSG format: "{LIN}:{ANG}:{SERVO_ANG}"
const char* host_name = "Minibot_ESP";
const char* ssid = "SSID Here";
const char* password = "PASS Here";

WiFiUDP port;
char packetBuffer[255];
unsigned int localPort = 9999;

//PID Tuning Paramaters
double Kp = 1.0, Ki = 0.0, Kd = 0.0;

//Desired Left and Right velocities based of Linear/Angular Velocity 
double v_right_setpoint;
double v_left_setpoint;

//Measured velocities of the motors
double v_right_input;
double v_left_input;

//Output Values to be sent to the Sabertooth
double right_output_value;
double left_output_value;


//Set up Left PID
PID Left_PID(&v_left_input, &left_output_value, &v_left_setpoint, Kp, Ki, Kd, DIRECT);

 //Set up Right PID
PID Right_PID(&v_right_input, &right_output_value, &v_right_setpoint, Kp, Ki, Kd, DIRECT);



//Combines the linear velocity and angular velocity into individual velocities for the left/right motor
void convert_to_linear(float ang_vel, float lin_vel, double& v_right, double& v_left){

    v_right = ((2.0*lin_vel) + (ang_vel*robot_width))/(2.0*wheel_radius);
    v_left  = ((2.0*lin_vel) - (ang_vel*robot_width))/(2.0*wheel_radius);    

}

void send_to_motor_driver(){

  //Conditions output to be in correct ranges for each motor

  //Sabertooth
  //(The only difference is the MSb is set for one, and not the other)
  //M1===>   1 to 127 | 64 = stop
  //M2===> 128 to 255 | 192 is stop

  //This is probably a horrible way of doing this, but I thought I would try it out.
  //Cast float => int => byte

  int right_output_int = (int)right_output_value;
  int left_output_int  = (int)left_output_value;

  byte right_output_byte = (byte)right_output_value;
  byte left_output_byte  = (byte)left_output_value;

  //Sets MSb of second motor
  left_output_byte = 0x80 | left_output_byte;

  //Sends values over serial to the Sabertooth
  Serial.write(right_output_byte);
  Serial.write(left_output_byte);

  
}

void Motor_Setup(){

  //Set output limits
  Left_PID.SetOutputLimits(min_out, max_out);

  Left_PID.SetMode(AUTOMATIC); //Turns PID On


  //Set output limits
  Right_PID.SetOutputLimits(min_out, max_out);

  Right_PID.SetMode(AUTOMATIC); //Turns PID On
}

void Motor_Update(float ang_vel, float lin_vel){


  //Convert angular + linear velocity commands to individual left/right velocity setpoints
  convert_to_linear(ang_vel, lin_vel, v_right_setpoint, v_left_setpoint);
    
  //Get/Set Input Varaible Values(Presumably from motor encoders) This should assign v_(right/left)_input in terms of a velocty(m/s)
  
  //===============to be filled in=================//

  //Compute Right/Left PID Output Values

  Right_PID.Compute();
  Left_PID.Compute();

  //Send values to motor driver
  send_to_motor_driver();

}



void setup() {
  // Initialize PWM pin, TODO: Verify pin mode correct!
  pinMode(SERVO_PWM_PIN, OUTPUT);

  // Debug Serial
  Serial1.begin(115200);

  WiFi.hostname(host_name);
  WiFi.begin(ssid, password);
  
  Serial1.print("\nConnecting to "); Serial1.println(ssid);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
  }
  // open UDP port
  port.begin(localPort);
  //start main Serial
  Serial.begin(115200);
  // Debug Serial
  Serial1.print("Ready!  IP: ");
  Serial1.print(WiFi.localIP());
  Serial1.print(" Port: ");
  Serial1.println(localPort);
  
  //Sets up Motor PIDs
  Serial1.print("Setting up PID controllers");
  Motor_Setup();
  Serial1.println("...Done.");

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
  lin = atof(val);
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
  ang = atof(val);
  i++;
  val[i] = '\0';

  // parse servo angle
  for(i; i<len; i++){
    val[j] = packetBuffer[i];
    j++;
  }
  val[j] = '\0';
  servo_ang = atof(val);
  
  // clear buffer
  packetBuffer[0] = '\0';
  
  // Debug output
  Serial1.println("Command parsed!");

  // Call PID/Servo Control Here
  Motor_Update(ang, lin);
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
    parse_cmd(len);
  }

  delay(25);
}
