#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
// #include <haptic_glove_ros>
#include "haptic_glove_ros/Vibration.h"
#include "vibrator_manager/vibrator_manager.h"
#include <sstream>

VibrationManager *vm;

void vibration_callback(const haptic_glove_ros::Vibration& msg)
{
    // ROS_INFO("%s", "Callback_called");
    vm->send_state(msg.levels_per_pin.data());
}

void onShutdown(int sig)
{
    vm->reset();
    ros::shutdown();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "vibrator_controller");
	vm = new VibrationManager(&vibration_callback);
  	ros::Rate loop_rate(30);   
    signal(SIGINT, onShutdown);
  	int count = 0;
  	// std_msgs::Float32 msg;

  	while (ros::ok())
  	{

    	ros::spinOnce();

    	loop_rate.sleep();
    	++count;
  }
  vm->reset();


  ros::spin();

	return 0;
}