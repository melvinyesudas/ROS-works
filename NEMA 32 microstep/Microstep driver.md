#  Stepper angle control using NEMA 32 microstep driver through ROS.
                Here is the video of controlling a stepper motor angle with a Microstep driver(NEMA 32) through ROS.
                            If we Input a -ve angle through terminal it rotate in Clockwise direction in the given angle,+ve means Counter clockwise direction.

# NEMA 32 configuring
  For 32 step and lower power consumption (less Torque).
   - SW1:-OFF
   - SW2:-OFF
   - SW3:-OFF
   - SW4:-ON 
   - SW5:-ON
   - SW6:-ON
   
# Ros-Arduino code

```

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
signed int angle;
const int stepPin = 9; 
const int dirPin = 8;
int x;

ros::NodeHandle  nh;
void pwm( const std_msgs::Int16& cmd_msg)
{
  
angle=cmd_msg.data;
 
 if (angle<0)
 {
 cw();
 delay(100);
 }
 
 else
 {
 ccw();
 delay(100);
 }
 
      
}


ros::Subscriber<std_msgs::Int16> sub("servo", pwm);

void setup()
{
   pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}
void loop()
{
  nh.spinOnce();
  delay(1);
}
void cw()
{
  digitalWrite(dirPin,LOW); 
  for(x = 0; x < 17.77*(angle*-1); x++)
  {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }}
  void ccw()
{
  digitalWrite(dirPin,HIGH); 
  for(x = 0; x < 17.77*angle; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }}
 
```
# ROS Commands

```
$ roscore
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
$ rostopic pub servo std_msgs/Int16 "data: -180" 

```






