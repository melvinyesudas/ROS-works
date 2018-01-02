# ROS-Ultrasonic
Using rosserial protocol object counting.


# ROS-Arduino code

```
 #include <ros.h>
   #include <ros/time.h>
    #include <sensor_msgs/Range.h>
   
  ros::NodeHandle  nh;
   sensor_msgs::Range range_msg;
   ros::Publisher pub_range( "range_data", &range_msg);
   
   const int analog_pin = 0;
   unsigned long range_timer;
  
   /*
  19  * getRange() - samples the analog input from the ranger
  20  * and converts it into meters.  
  21  * 
  22  * NOTE: This function is only applicable to the GP2D120XJ00F !!
  23  * Using this function with other Rangers will provide incorrect readings.
  24  */
   float getRange(int pin_num){
       int sample;
       // Get data
       sample = analogRead(pin_num)/4;
       // if the ADC reading is too low, 
       //   then we are really far away from anything
       if(sample < 10)
           return 254;     // max range
      // Magic numbers to get cm
       sample= 1309/(sample-3);
       return (sample - 1)/100; //convert to meters
   }
   
   char frameid[] = "/ir_ranger";
   
   void setup()
   {
     nh.initNode();
     nh.advertise(pub_range);
     
     range_msg.radiation_type = sensor_msgs::Range::INFRARED;
     range_msg.header.frame_id =  frameid;
     range_msg.field_of_view = 0.01;
     range_msg.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
     range_msg.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
   }
   
   void loop()
   {
   // publish the range value every 50 milliseconds
     //   since it takes that long for the sensor to stabilize
    if ( (millis()-range_timer) > 50){
       range_msg.range = getRange(analog_pin);
       range_msg.header.stamp = nh.now();
       pub_range.publish(&range_msg);
      range_timer =  millis();
     }
     nh.spinOnce();
   }
```
# ROS-commands

```
$ roscore
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
$ rostopic echo rangedata
```


