#define NUM_PINS 6
#define NUM_LEVELS 5
#define BAUD_RATE 57600 


class GloveDriver{
 
 public:
   GloveDriver(void callback(const int));
   const static int vibePins[];
   const static int vibeLevel[];
   const static char pin_names[][10];
   void make_vib();
   void invert_blink();
   void synch();
   void spin();
   void reset();
   int get_npins(){return num_pins;}
   int get_nlevels(){return num_vlevels;}
   void set_pin(int pin, int value);
   int get_pin(int pin);
   
 private:
//    const char* topic_name = "vibration_command";
   const int baud_rate = BAUD_RATE;
   const static int num_pins;
   const static int num_vlevels;
   bool blinker = false;
   int pin_state[NUM_PINS];
   void (*cb)(const int);

   
 };
 
const int GloveDriver::vibePins[] = {5, 15, 12, 11, 6, 9};
const int GloveDriver::vibeLevel[] = {0, 140, 170, 200, 230};
const char GloveDriver::pin_names[][10] = {"Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"};
const int GloveDriver::num_pins = sizeof(vibePins) / sizeof(*vibePins);
const int GloveDriver::num_vlevels = sizeof(vibeLevel) / sizeof(*vibeLevel);
//int GloveDriver::pin_state[num_pins] = {0, 0, 0, 0, 0, 0};

GloveDriver::GloveDriver(void (*callback)(const int)){
  Serial.begin(baud_rate);
//  Serial.print("number of pins: ");
//  Serial.println(num_pins);
//  Serial.print("number of vibration levels: ");
//  Serial.println(num_vlevels);
  cb = callback;
  pinMode(13, OUTPUT);
  for(int i=0; i<num_pins; i++){
     pinMode(vibePins[i], OUTPUT);
   }
 reset();

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

//void GloveDriver::synch()
//{
//  Serial.print("[SYNCH] ");
//  Serial.print("number of pins: ");
//  Serial.print(num_pins);
//  Serial.print(" \t number of vibration levels: ");
//  Serial.println(num_vlevels);
//  
//};
//  
void GloveDriver::make_vib(){
 if(pin_state != NULL){
       String res_string("Active Pins: {");
       String temp = String("");
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

         Serial.print("[STATUS] ");
         Serial.println(res_string.c_str());

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
   Serial.print("[ERROR] ");
   Serial.println(temp.c_str());
   return;
   } 

 if(pin_idx < 0 or pin_idx > num_pins)
 {
   temp = "The value provided for the pin idx is not valid   -> " + String(pin_idx);
   Serial.println("[ERROR] ");
   Serial.print(temp.c_str());
   return;
   } 
 pin_state[pin_idx] = level_idx;
};

int GloveDriver::get_pin(int pin_idx){
 return pin_state[pin_idx];
};



void GloveDriver::spin(){
 String x;
 int command = -1;
 x = Serial.readString();
 int pattern_start = x.startsWith("[MSG]");
 if(x.startsWith("[MSG]")){
  command = x.substring(6).toInt();
  
//  Serial.println(String("[INFO] Toggling ") + String(pin_names[x.toInt() / get_nlevels()]) + String("..."));
  this->cb(command);
  }
  else
  {
    Serial.println(String("[ERROR] Invalid command "));
   }
};





GloveDriver *gd;

void callback(const int msg){
 gd->invert_blink();
//  Serial.println(msg.data);
 int value = msg;
 int vlevel_idx = value % gd->get_nlevels(); 
 int pin_idx = value / gd->get_nlevels();
 if(gd->get_pin(pin_idx) != vlevel_idx)
   {
     gd->set_pin(pin_idx, vlevel_idx);
     gd->make_vib();
   }
   
   gd->invert_blink();
 };





int x;

void setup()
{
Serial.begin(BAUD_RATE);
Serial.setTimeout(1);
gd = new GloveDriver(&callback);
};


void loop(){
  while(! Serial.available()){
    gd->invert_blink();
    delay(30);
    }


  gd->spin();
  };
