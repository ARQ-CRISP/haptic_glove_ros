#include "vibrator_manager/vibrator_manager.h"


const char* VibrationManager::listening_topic_name = "vibration_state";
const char* VibrationManager::publishing_topic_name = "vibration_command";

VibrationManager::VibrationManager(void callback(const haptic_glove_ros::Vibration&))
{
    pub = nh.advertise<std_msgs::Int32>(publishing_topic_name, 100);
    sub = nh.subscribe(listening_topic_name, 1, callback);
};

void VibrationManager::send_state()
{
    for (int i=0; i<haptic_glove_ros::Vibration::n_pins; i++)
    {
        int value = vibration_state[i] + i * haptic_glove_ros::Vibration::n_levels;
        std_msgs::Int32 new_msg; 
        new_msg.data = value;
        pub.publish(new_msg);
    }

};

void VibrationManager::update_state(const short unsigned int* state)
{
    for (int i=0; i<haptic_glove_ros::Vibration::n_pins; i++)
    {
        vibration_state[i] = state[i];
    }

};

void VibrationManager::reset()
{
    unsigned short base_state[] = {0, 0, 0, 0, 0, 0};
    update_state(base_state);
    send_state();
};

