#include <ros.h>
#include <std_msgs/Int32.h>
//#include <SimbleeCOM.h>

class GloveDriver{
  
  public:
    GloveDriver(void callback(const std_msgs::Int32&));
    const static int vibePins[];
    const static int vibeLevel[];
    void make_vib(unsigned int, unsigned int );
    void invert_blink();
    void spin();
    int get_npins(){return num_pins;}
    int get_nlevels(){return num_vlevels;}
  private:
    const char* topic_name = "vibration_command";
    const int baud_rate = 57600;
    int num_pins = 0;
    int num_vlevels = 0;
    bool blinker = false;
    ros::NodeHandle nh;
    ros::Subscriber<std_msgs::Int32> *sub;
    
  };
const int GloveDriver::vibePins[] = {5, 15, 12, 11, 6, 9};
const int GloveDriver::vibeLevel[] = {0, 140, 170, 200, 230};

GloveDriver::GloveDriver(void callback(const std_msgs::Int32&)){
   Serial.begin(baud_rate);
   num_pins = sizeof(vibePins) / sizeof(*vibePins);
   num_vlevels = sizeof(vibeLevel) / sizeof(*vibeLevel);
   Serial.print("number of pins: ");
   Serial.println(num_pins);
   Serial.print("number of vibration levels: ");
   Serial.println(num_vlevels);
   
   pinMode(13, OUTPUT);
   for(int i=0; i<num_pins; i++){
      pinMode(vibePins[i], OUTPUT);
    }
  
  nh.initNode(); //haptic_glove_listener
  sub = new ros::Subscriber<std_msgs::Int32>(topic_name, callback);
  nh.subscribe(*sub);
  };
  
void GloveDriver::invert_blink()
{
  blinker = not blinker;
  if(blinker){
     digitalWrite(13, HIGH);
    }
  else{
      digitalWrite(13, LOW);
      }
  };
  
void GloveDriver::make_vib(unsigned int vib_idx, unsigned int pin_idx){
  String temp;
  if(vib_idx < 0 or vib_idx > num_vlevels)
  {
    temp = "The value provided for the vibration level is not valid  -> " + String(vib_idx);
    nh.logerror(temp.c_str());
    return;
    } 

  if(pin_idx < 0 or pin_idx > num_pins)
  {
    temp = "The value provided for the pin idx is not valid   -> " + String(vib_idx);
    nh.logerror(temp.c_str());
    return;
    } 
  analogWrite(vibePins[pin_idx], vibeLevel[vib_idx]);
  nh.loginfo("Vibration succeded!");
  
  temp = (String("Vibrating pin: ") + String(pin_idx));
  nh.loginfo(temp.c_str());
  temp = (String("Vibrating intensity: ") + String(vib_idx));
  nh.loginfo(temp.c_str());
  };

void GloveDriver::spin(){
  nh.spinOnce();
};





GloveDriver *gd;

void callback(const std_msgs::Int32& msg){
  gd->invert_blink();
  Serial.println(msg.data);
//  Serial.print("\n");
  int value = msg.data;
  int vlevel_idx = value % gd->get_nlevels(); 
  int pin_idx = value / gd->get_nlevels();

  Serial.print("vlevel : ");
  Serial.println(vlevel_idx);
  Serial.print("pin_idx : ");
  Serial.println(pin_idx);
  gd->make_vib(vlevel_idx, pin_idx);
  gd->invert_blink();
  };


void setup() {
  // put your setup code here, to run once:
  
  gd = new GloveDriver(&callback);
  Serial.println("Initialization Complete");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(100);
  gd->spin();
//  gd->invert_blink();

}
