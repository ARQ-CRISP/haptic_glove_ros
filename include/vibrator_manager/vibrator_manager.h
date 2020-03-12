#ifndef VIBRATION_MANAGER_H
#define VIBRATION_MANAGER_H

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "haptic_glove_ros/Vibration.h"


class VibrationManager
{
    public:
        VibrationManager(void callback(const haptic_glove_ros::Vibration&));
        void send_state(const short unsigned int*);
        void reset();

    private:
        const static char* listening_topic_name;
        const static char* publishing_topic_name;

        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
};


#endif