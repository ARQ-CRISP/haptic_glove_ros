
#include "ros/ros.h"
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
private:
const static char* listening_topic_name;
const static char* publishing_topic_name;
ros::NodeHandle nh;
// ros::Subscriber *sub;
ros::Subscriber sub;
ros::Publisher pub;
// ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
};

const char* VibrationManager::listening_topic_name = "vibration_state";
const char* VibrationManager::publishing_topic_name = "vibration_command";
// GloveDriver::GloveDriver(void callback(const std_msgs::Int32&))
VibrationManager::VibrationManager(void callback(const haptic_glove_ros::Vibration&))
{
// nh.initNode(); //haptic_glove_listener
// sub = new ros::Subscriber(listening_topic_name, callback);
pub = nh.advertise<std_msgs::Int32>(publishing_topic_name, 100);
sub = nh.subscribe(listening_topic_name, 100, callback);
// nh.subscribe(sub);
};

void VibrationManager::send_state(const short unsigned int* state)
{
    for (int i=0; i<haptic_glove_ros::Vibration::n_pins; i++){
        int value = state[i] + i * haptic_glove_ros::Vibration::n_levels;
        std_msgs::Int32 new_msg; 
        new_msg.data = value;
        pub.publish(new_msg);
        }

}



VibrationManager *vm;

void vibration_callback(const haptic_glove_ros::Vibration& msg)
{
    ROS_INFO("%s", "Callback_called");
    vm->send_state(msg.levels_per_pin.data());
}



int main(int argc, char **argv){
	ros::init(argc, argv, "vibrator_controller");
	vm = new VibrationManager(&vibration_callback);
    haptic_glove_ros::Vibration *v;
  	ros::Rate loop_rate(10);   

  	int count = 0;
  	// std_msgs::Float32 msg;

  	while (ros::ok())
  	{
  		// messageVariable = fingerVariable + forceVariable;
    	// msg.data = messageVariable;	
  			
    	// chatter_pub.publish(msg);

    	ros::spinOnce();

    	loop_rate.sleep();
    	++count;
  }


  ros::spin();

	return 0;
}