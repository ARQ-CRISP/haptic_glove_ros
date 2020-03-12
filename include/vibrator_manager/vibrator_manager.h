#ifndef VIBRATION_MANAGER_H
#define VIBRATION_MANAGER_H

#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
// #include <haptic_glove_ros>
#include "haptic_glove_ros/Vibration.h"
#include <sstream>


class VibrationManager
{
public:
    // GloveDriver(void callback(const std_msgs::Int32&));
VibrationManager(void callback(const haptic_glove_ros::Vibration&));
void send_state(const short unsigned int*);
void reset();
private:
const static char* listening_topic_name;
const static char* publishing_topic_name;
ros::NodeHandle nh;
// ros::Subscriber *sub;
ros::Subscriber sub;
ros::Publisher pub;
// ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
};


#endif