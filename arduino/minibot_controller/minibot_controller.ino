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

// pin assignments
#define SERVO_PWM_PIN 14
#define ENC_LEFT_A 4
#define ENC_LEFT_B 5
#define ENC_RIGHT_A 12
#define ENC_RIGHT_B 13

#define ETX '\r'
#define STX 'B'

//Robot Physical Paramaters
#define WHEEL_RADIUS 2.0
#define ROBOT_WIDTH  1.0
#define ENC_CPR 768

//Output limits for PID. The second motor is set to proper range before sending to motor driver(see function send_to_motor_driver() for details)
#define MIN_OUTPUT 1.0
#define MAX_OUTPUT 128.0

// Using Netstrings format: length:msg,
// MSG format: "{LIN}:{ANG}:{SERVO_ANG}"
const char* host_name = "Minibot_ESP";
const char* ssid = "SSID Here";
const char* password = "PASS Here";

// UDP info
WiFiUDP port;
char packetBuffer[255];
unsigned int localPort = 9999;

// Command values
float cmd_lin_vel, cmd_ang_vel, cmd_servo_ang;

//PID Tuning Paramaters
double Kp = 1.0, Ki = 0.0, Kd = 0.0;

//Desired Left and Right velocities based of Linear/Angular Velocity 
double v_right_setpoint;
double v_left_setpoint;

//Measured velocities of the motors
double v_right_input = 0;
double v_left_input = 0;

//Output Values to be sent to the Sabertooth
double right_output_value;
double left_output_value;

// Encoder velocity variables
volatile long rEncVal = 0;
volatile long lEncVal = 0;
long newLPos, newRPos;
long oldLPos = 0;
long oldRPos = 0;
unsigned long curTime;
unsigned long pastTime = 0;

//Set up Left PID
PID Left_PID(&v_left_input, &left_output_value, &v_left_setpoint, Kp, Ki, Kd, DIRECT);
 //Set up Right PID
PID Right_PID(&v_right_input, &right_output_value, &v_right_setpoint, Kp, Ki, Kd, DIRECT);


//Combines the linear velocity and angular velocity into individual velocities for the left/right motor
void convert_to_linear(float ang_vel, float lin_vel, double& v_right, double& v_left){

    v_right = ((2.0*lin_vel) + (ang_vel*ROBOT_WIDTH))/(2.0*WHEEL_RADIUS);
    v_left  = ((2.0*lin_vel) - (ang_vel*ROBOT_WIDTH))/(2.0*WHEEL_RADIUS);    

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

  byte right_output_byte = (byte)right_output_int;
  byte left_output_byte  = (byte)left_output_int;

  //Sets MSb of second motor
  left_output_byte = 0x80 | left_output_byte;

  //Sends values over serial to the Sabertooth
  Serial.write(right_output_byte);
  Serial.write(left_output_byte);

  
}

void Motor_Setup(){

  //Set output limits
  Left_PID.SetOutputLimits(MIN_OUTPUT, MAX_OUTPUT);

  Left_PID.SetMode(AUTOMATIC); //Turns PID On


  //Set output limits
  Right_PID.SetOutputLimits(MIN_OUTPUT, MAX_OUTPUT);

  Right_PID.SetMode(AUTOMATIC); //Turns PID On
}

void Motor_Update(){


  //Convert angular + linear velocity commands to individual left/right velocity setpoints
  convert_to_linear(cmd_ang_vel, cmd_lin_vel, v_right_setpoint, v_left_setpoint);
    
  //Get/Set Input Varaible Values(Presumably from motor encoders) This should assign v_(right/left)_input in terms of a velocty(m/s)
  
  sample_Vels();

  //Compute Right/Left PID Output Values

  Right_PID.Compute();
  Left_PID.Compute();

  //Send values to motor driver
  send_to_motor_driver();

}



void setup() {
  // Initialize pins
  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);
  pinMode(SERVO_PWM_PIN, OUTPUT);

  // enable pullups
  digitalWrite(ENC_LEFT_A, HIGH);
  digitalWrite(ENC_LEFT_B, HIGH);
  digitalWrite(ENC_RIGHT_A, HIGH);
  digitalWrite(ENC_RIGHT_B, HIGH);

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

  // Set Pin interrupts
  attachInterrupt(ENC_LEFT_A, l_pin_chng, RISING);
  attachInterrupt(ENC_RIGHT_A, r_pin_chng, RISING);
  Serial1.println("Encoder pin interrupts enabled...");
}

void parse_cmd(int len)
{
  int i = 0;
  int j = 0;
  char val[32];
  char current;

  // parse linear
  while (packetBuffer[i] != ':'){
    val[j] = packetBuffer[i];
    j++;
    i++;
  }
  val[j] = '\0';
  cmd_lin_vel = atof(val);
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
  cmd_ang_vel = atof(val);
  i++;
  val[i] = '\0';

  // parse servo angle
  for(i; i<len; i++){
    val[j] = packetBuffer[i];
    j++;
  }
  val[j] = '\0';
  cmd_servo_ang = atof(val);
  
  // clear buffer
  packetBuffer[0] = '\0';
  
  // Debug output
  Serial1.println("Command parsed!");
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
  
  Motor_Update();
  delay(25);
}

void sample_Vels() {
  newLPos = lEncVal;
  newRPos = rEncVal;
  curTime = millis();
  v_right_input = ( newRPos - oldRPos ) * 1000.0 / ( curTime - pastTime ) / ENC_CPR * WHEEL_RADIUS ; // m/sec
  v_left_input = ( newLPos - oldLPos ) * 1000.0 / ( curTime - pastTime ) / ENC_CPR * WHEEL_RADIUS ; // m/sec
  pastTime = curTime;
  oldRPos = newRPos;
  oldLPos = newLPos;
}
void l_pin_chng() {
  // Triggers on rising A, so A is HIGH
  int encoded = 0b10 | digitalRead(ENC_LEFT_B);

  int sum = ( lEncVal << 2 ) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) lEncVal ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) lEncVal --;

  lEncVal = encoded;
}

void r_pin_chng() {
  // Triggers on rising A, so A is HIGH
  int encoded = 0b10 | digitalRead(ENC_RIGHT_B);

  int sum = ( rEncVal << 2 ) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rEncVal ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rEncVal --;

  rEncVal = encoded;
}

