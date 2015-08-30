#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>
#include <motorcontrol.h>
#include <hotwheels_msgs/Control.h>
#include <hotwheels_msgs/Feedback.h>

ros::NodeHandle nh;

Motorcontrol m1;
Motorcontrol m2;
int m1pwmmsg=0;
int m2pwmmsg=0;
int m1gpiomsg=0;
int m2gpiomsg=0;

std_msgs::String str_msg;
hotwheels_msgs::Feedback feedback_msg_var;

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher feedback("feedback", &feedback_msg_var);

char hello[13] = "hello world!";

float valuetrimmer(float value){
  float trimmedvalue=value; 
  if (value<3.0){
     trimmedvalue=1.0;
   } else if(value>75.0){
     trimmedvalue=75.0;
   }
   return trimmedvalue;
}

void messageControlExec( const hotwheels_msgs::Control& controldata)
{
  feedback_msg_var.act_vel[0]=controldata.vel_commands[0];
  feedback_msg_var.act_vel[1]=controldata.vel_commands[1];
  feedback_msg_var.act_vel[2]=controldata.vel_commands[2];
  feedback_msg_var.act_vel[3]=controldata.vel_commands[3];
  
  float m1pwmvalue=controldata.vel_commands[0];
  float m2pwmvalue=controldata.vel_commands[2];
  
 if (controldata.vel_commands[0]<0){
    digitalWrite(PF_2, HIGH);
    m1pwmvalue=-1.0*m1pwmvalue;
  }else{
    digitalWrite(PF_2, LOW);
 }
 if (controldata.vel_commands[2]<0){
    digitalWrite(PF_4, LOW);
    m2pwmvalue=-1.0*m2pwmvalue;
  }else{
     digitalWrite(PF_4, HIGH);
 }
 m1pwmvalue=valuetrimmer(m1pwmvalue);
 m2pwmvalue=valuetrimmer(m2pwmvalue);
 
 feedback_msg_var.act_vel[0]=m1pwmvalue;
 feedback_msg_var.act_vel[2]=m2pwmvalue;
 feedback.publish( &feedback_msg_var );
 m1.writePercent(m1pwmvalue);
 m2.writePercent(m2pwmvalue);
}

ros::Subscriber<hotwheels_msgs::Control> subcontroltopic("/tiva/control_msg", &messageControlExec );

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(feedback);
  nh.subscribe(subcontroltopic);
  //m1.attach(PB_4, 20, 8000);
  m1.attach(PB_4, 8000);
  m1.writePercent(1);
  m2.attach(PB_6, 8000);
  m2.writePercent(1);
  pinMode(PF_2, OUTPUT); 
  digitalWrite(PF_2, HIGH);
  pinMode(PF_4, OUTPUT); 
  digitalWrite(PF_4, HIGH);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  //m1.writePercent(10);
  // delay(100);
}