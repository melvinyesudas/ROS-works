```

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#define encoder0PinA  2  //CLK Output A Do not use other pin for clock as we are using interrupt
#define encoder0PinB  4  //DT Output B

 
volatile signed int encoder0Pos = 0;
 
   
   ros::NodeHandle  nh;
   sensor_msgs::Range range_msg;
   ros::Publisher pub_range( "range_data", &range_msg);
   
 
   unsigned long range_timer;
  
   char frameid[20] = "/encoder_rotation";
   
   void setup()
   
   {
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);      
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);      
  attachInterrupt(0, doEncoder, RISING);

     nh.initNode();
     nh.advertise(pub_range);
   
   }
   
   void loop()
   {
       
    if ( (millis()-range_timer) > 50)
      {
        
       range_msg.range = encoder0Pos/360 ;
       range_msg.max_range = encoder0Pos;
       range_msg.min_range = encoder0Pos%360;
           
  
       pub_range.publish(&range_msg);

     }
  
     nh.spinOnce();
   }
    
void doEncoder() {
  if (digitalRead(encoder0PinB)==HIGH) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }}

```
