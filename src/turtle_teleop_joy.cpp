// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
using namespace std;
// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%

union Deviceb
{
  struct
  {
    unsigned char right_trig : 1;
    unsigned char left_trig : 1;
    unsigned char right_but : 1;
    unsigned char left_but : 1;
    unsigned char bit5 : 1;
    unsigned char bit6 : 1;
    unsigned char bit7 : 1;
    unsigned char bit8 : 1;
  };
  unsigned char status;
};

union Xpad{
   struct{
   short a,b;
   union Deviceb buttons;
   };
   char xpadstate[2*sizeof(short)+sizeof(char)];
};

class TeleopTurtle
{
public:
  TeleopTurtle();
  ~TeleopTurtle();

   int fd;
 char * myfifo;

  float scale(float DesieredMax ,float DesieredMin, float PIDOut, float MinimumValue, float MaximumValue)
{

    return (DesieredMax-DesieredMin)*(PIDOut-MinimumValue)/(MaximumValue-MinimumValue)+DesieredMin;

}
  




private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int left_stick,right_stick;
  //ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};
// %EndTag(CLASSDEF)%
// %Tag(PARAMS)%
TeleopTurtle::~TeleopTurtle(){
    //close(newsockfd);
    // close(sockfd);
close(fd);
}

TeleopTurtle::TeleopTurtle():left_stick(1),right_stick(4)
{


  //nh_.param("left_y_stick", left_stick, left_stick);
  //nh_.param("right_y_stick", right_stick, right_stick);
// %EndTag(PARAMS)%
// %Tag(PUB)%
  //vel_pub_ = nh_.advertise<turtlesim::Twist>("turtle1/command_Twist", 1);
// %EndTag(PUB)%
// %Tag(SUB)%
//server code
  myfifo = new char [12];
 strcpy(myfifo, "./myfifo1");

 /* create the FIFO (named pipe) */
 mkfifo(myfifo, 0600);
 /* write "Hi" to the FIFO */
 fd = open("./myfifo1", O_RDWR ); //open(myfifo, O_WRONLY | O_NONBLOCK);
 if (fd == -1) {
     perror("open");
     exit(0);
 } 
 printf("File open\n");





  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/control_station/joy", 10, &TeleopTurtle::joyCallback, this);
// %EndTag(SUB)%
}


// %Tag(CALLBACK)%
void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  char buf[8];
  union Xpad gamepad;
  //turtlesim::Twist vel;
  //vel.angular = a_scale_*joy->axes[angular_];
  //vel.linear = l_scale_*joy->axes[linear_];
  //vel_pub_.publish(vel);
  //cout<<joy->axes[1]<<" "<<sizeof(float)<<endl;
  gamepad.a = (short)scale(32767,0,joy->axes[left_stick],0,1);
  gamepad.b = (short)scale(32767,0,joy->axes[right_stick],0,1);
  
  gamepad.buttons.right_trig=(int)joy->buttons[0];


  
  gamepad.buttons.left_trig=(int)joy->buttons[1];

  gamepad.buttons.left_but=(int)joy->buttons[4];
  gamepad.buttons.right_but=(int)joy->buttons[5];

  cout<<gamepad.a<<" "<<gamepad.b<<" "<< (int)gamepad.buttons.right_trig <<" "<<(int) gamepad.buttons.right_but <<" "<< (int)gamepad.buttons.left_trig << " " <<(int)gamepad.buttons.left_but<<endl;
  //bzero(buf,8);
  //memcpy(buf,&a,sizeof(float));
  //n = write(this->newsockfd,gamepad.xpadstate,sizeof(Xpad));
  write(fd, gamepad.xpadstate, sizeof(Xpad));
  
}
// %EndTag(CALLBACK)%
// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;
 while(ros::ok()){
  ros::spinOnce();
}

 
}
// %EndTag(MAIN)%
// %EndTag(FULL)%

