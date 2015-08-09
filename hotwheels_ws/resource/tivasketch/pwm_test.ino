#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>
#include <motorcontrol.h>
ros::NodeHandle nh;

Motorcontrol m1;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";
void messageCb( const std_msgs::Byte& pwm)
{
  if (pwm.data<0){
    
    m1.writePercent(0);
  }else if (pwm.data>80){
    
    m1.writePercent(80);
  }else{
    m1.writePercent(pwm.data);
  }
}

ros::Subscriber<std_msgs::Byte> sub("motor_pwm_input", &messageCb );
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  //m1.attach(PB_4, 20, 8000);
  m1.attach(PB_4, 8000);
  pinMode(PF_2, OUTPUT); 
  digitalWrite(PF_2, HIGH);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  for (int i=10;i<50;i++){
    
  m1.writePercent(i); 
  delay(100); 
}
for (int i=50;i>10;i--){
    
  m1.writePercent(i); 
  delay(100); 
}
  
 
  
}
