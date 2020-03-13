#ifndef VIBRATION_MANAGER_H
#define VIBRATION_MANAGER_H

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "haptic_glove_ros/Vibration.h"


class VibrationManager
{
    public:
        VibrationManager(void callback(const haptic_glove_ros::Vibration&));
        void update_state(const short unsigned int*);
        void send_state();
        void reset();

    private:
        const static char* listening_topic_name;
        const static char* publishing_topic_name;

        int vibration_state[6] = {0, 0, 0, 0, 0, 0};
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
};


#endif