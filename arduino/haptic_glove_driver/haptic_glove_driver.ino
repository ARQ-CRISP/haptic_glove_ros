#include <ros.h>
#include <std_msgs/Int32.h>
#define NUM_PINS 6
#define NUM_LEVELS 5
#define BAUD_RATE 250000 
//57600

class GloveDriver{
  
  public:
    GloveDriver(void callback(const std_msgs::Int32&));
    const static int vibePins[];
    const static int vibeLevel[];
    const static char pin_names[][10];
    void make_vib();
    void invert_blink();
    void spin();
    void reset();
    int get_npins(){return num_pins;}
    int get_nlevels(){return num_vlevels;}
    void set_pin(int pin, int value);
    int get_pin(int pin);
    
  private:
    const char* topic_name = "vibration_command";
    const int baud_rate = BAUD_RATE;
    const static int num_pins;
    const static int num_vlevels;
    bool blinker = false;
    int pin_state[NUM_PINS];
    ros::NodeHandle nh;
    ros::Subscriber<std_msgs::Int32> *sub;
    
  };
  
const int GloveDriver::vibePins[] = {5, 15, 12, 11, 6, 9};
const int GloveDriver::vibeLevel[] = {0, 140, 170, 200, 230};
const char GloveDriver::pin_names[][10] = {"Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"};
const int GloveDriver::num_pins = sizeof(vibePins) / sizeof(*vibePins);
const int GloveDriver::num_vlevels = sizeof(vibeLevel) / sizeof(*vibeLevel);
//int GloveDriver::pin_state[num_pins] = {0, 0, 0, 0, 0, 0};

GloveDriver::GloveDriver(void callback(const std_msgs::Int32&)){
   Serial.begin(baud_rate);
   nh.getHardware().setBaud(baud_rate)
   Serial.print("number of pins: ");
   Serial.println(num_pins);
   Serial.print("number of vibration levels: ");
   Serial.println(num_vlevels);
   
   pinMode(13, OUTPUT);
   for(int i=0; i<num_pins; i++){
      pinMode(vibePins[i], OUTPUT);
    }
  reset();
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
  
void GloveDriver::make_vib(){
  if(pin_state != NULL){
        String res_string("Active Pins: {");
        String temp = NULL; // String("")
        for (int i=0; i<num_pins; i++)
        {
          //analogWrite(vibePins[i], vibeLevel[pin_state[i]]);  
          
          if (pin_state[i] > 0)
          {
            digitalWrite(vibePins[i], HIGH);  
            temp = String(pin_names[i]) + ": " + String(pin_state[i]) + ", ";
            res_string += temp;
            }
            else digitalWrite(vibePins[i], LOW); 
            
          }
        res_string += "}";
        if (temp != NULL)
        {
          nh.loginfo(res_string.c_str());
          }
  }

  };

void GloveDriver::reset()
{
  for (int i=0; i<num_pins; i++)
  {
    pin_state[i] = 0;
    }
  }
  
void GloveDriver::set_pin(int pin_idx, int level_idx){
  String temp("");
  if(level_idx < 0 or level_idx > num_vlevels)
  {
    temp = "The value provided for the vibration level is not valid  -> " + String(level_idx);
    nh.logerror(temp.c_str());
    return;
    } 

  if(pin_idx < 0 or pin_idx > num_pins)
  {
    temp = "The value provided for the pin idx is not valid   -> " + String(pin_idx);
    nh.logerror(temp.c_str());
    return;
    } 
  pin_state[pin_idx] = level_idx;
};

int GloveDriver::get_pin(int pin_idx){
  return pin_state[pin_idx];
};



void GloveDriver::spin(){
  nh.spinOnce();
};





GloveDriver *gd;

void callback(const std_msgs::Int32& msg){
  gd->invert_blink();
//  Serial.println(msg.data);
  int value = msg.data;
  int vlevel_idx = value % gd->get_nlevels(); 
  int pin_idx = value / gd->get_nlevels();
  if(gd->get_pin(pin_idx) != vlevel_idx)
    {
      gd->set_pin(pin_idx, vlevel_idx);
      gd->make_vib();
    }
    
    gd->invert_blink();
  };


void setup() {
  // put your setup code here, to run once:
  
  gd = new GloveDriver(&callback);
  Serial.println("Initialization Complete");
};

void loop() {
  // put your main code here, to run repeatedly:
  
  gd->invert_blink();
  delay(10);
  gd->spin();
  

}
