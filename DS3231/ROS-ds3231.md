# ROS-DS3231
Using rosserial protocol printing the real time in ros-terminal and save the data.

# Setting-up

![rtc](https://drive.google.com/open?id=1otZTCuKmVugRkdxvtusPQk8s8IvzB4F7)

# ROS-Arduino code

```
#include <ros.h>
#include <std_msgs/String.h>
#include <DS3231.h>

// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void setup()
{
   
  nh.initNode();
  nh.advertise(chatter);
   rtc.begin();
}

void loop()
{
 
  str_msg.data = rtc.getTimeStr();
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
```
# ROS-commands

```
$ roscore
$ rosrun rosserial_python serial_node.py /dev/ttyUSB0
$ rostopic echo chatter
//to save the data
$ mkdir ~/bagfiles
$ cd ~/bagfiles
$ rosbag record -a

```


