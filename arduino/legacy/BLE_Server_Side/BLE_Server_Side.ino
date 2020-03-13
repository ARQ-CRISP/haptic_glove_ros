#include <Wire.h>

#include <Console.h>
#include <FileIO.h>
#include <HttpClient.h>
#include <Process.h>
#include <YunServer.h>
#include <Mailbox.h>
#include <YunClient.h>


#include <SimbleeCOM.h>
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;
char toggleOff[] = {12, 11, 6};
const int vibePins[] = {5,15,12,11,6,9};

void messageCb( const std_msgs::Int32& toggle_msg){
  Serial.println(toggle_msg.data); 
  if (toggle_msg.data == 5){
  digitalWrite(13,HIGH);   // blink the led
  toggleOff[0] = 5;
  }
  else if(toggle_msg.data == 15){ 
  digitalWrite(13,HIGH);
  toggleOff[0] = 15;
  }
  else if(toggle_msg.data == 12){ 
  digitalWrite(13,HIGH);
  toggleOff[0] = 12;
  }
  else if(toggle_msg.data == 11){ 
  digitalWrite(13,HIGH);
  toggleOff[0] = 11;
  }
  else if(toggle_msg.data == 6){ 
  digitalWrite(13,HIGH);
  toggleOff[0] = 6;
  }
  else if(toggle_msg.data == 9){ 
  digitalWrite(13,HIGH);
  toggleOff[0] = 9;
  }
  else if(toggle_msg.data == 0){ 
  digitalWrite(13,LOW);
  toggleOff[0] = 0;
  }
}

ros::Subscriber<std_msgs::Int32> sub("ble_com", &messageCb );

void setup()
{ 
  pinMode(13,OUTPUT);
  pinMode(5, OUTPUT);
  SimbleeCOM.txPowerLevel = 4;
  SimbleeCOM.proximityMode(0);
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  SimbleeCOM.begin();
}

void loop()
{  
  nh.spinOnce();
  
  digitalWrite(13,LOW);
  delay(100);
  digitalWrite(13,HIGH);
  delay(100);
  delay(1);
  //toggleOff[0] = 11;
  SimbleeCOM.send(toggleOff,sizeof(toggleOff));
  
}
