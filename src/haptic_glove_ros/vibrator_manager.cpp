
#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
// #include <haptic_glove_ros>
#include "haptic_glove_ros/Vibration.h"
#include "vibrator_manager/vibrator_manager.h"
#include <sstream>


const char* VibrationManager::listening_topic_name = "vibration_state";
const char* VibrationManager::publishing_topic_name = "vibration_command";
// GloveDriver::GloveDriver(void callback(const std_msgs::Int32&))
VibrationManager::VibrationManager(void callback(const haptic_glove_ros::Vibration&))
{
// nh.initNode(); //haptic_glove_listener
// sub = new ros::Subscriber(listening_topic_name, callback);
pub = nh.advertise<std_msgs::Int32>(publishing_topic_name, 100);
sub = nh.subscribe(listening_topic_name, 100, callback);
// reset();
};

void VibrationManager::send_state(const short unsigned int* state)
{
    for (int i=0; i<haptic_glove_ros::Vibration::n_pins; i++){
        int value = state[i] + i * haptic_glove_ros::Vibration::n_levels;
        std_msgs::Int32 new_msg; 
        new_msg.data = value;
        pub.publish(new_msg);
        }

};

void VibrationManager::reset()
{
    unsigned short base_state[] = {0, 0, 0, 0, 0, 0};
    send_state(base_state);
};

