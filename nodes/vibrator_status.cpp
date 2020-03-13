#include <signal.h>
#include "vibrator_manager/vibrator_manager.h"

VibrationManager *vm;

void vibration_callback(const haptic_glove_ros::Vibration& msg)
{
    vm->update_state(msg.levels_per_pin.data());
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
  	while (ros::ok())
  	{
		
    	vm->send_state();
		loop_rate.sleep();
    	ros::spinOnce();
    }
    vm->reset();
    ros::spin();
	return 0;
}
