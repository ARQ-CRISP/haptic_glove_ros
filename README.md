# haptic-glove-ros
Repository containing the code to communicate with the haptic-glove




Check repo [readme](https://github.com/samisnotinsane/arq-teleop-robot/tree/bura_de)



To use this repo:

- Install Arduino IDE 1.6.8 or newer;
- open the haptic_glove_driver sketch with it and go to File->Preferences;
- In `Additional Boards Manager URLs:` insert https://raw.githubusercontent.com/OpenHAK/Simblee/master/package_openhak_index.json;
- go to Tools->Board and select the `BoardManager`. Thus search for the `OpenHAK` boards; 
- go to Tools->Board and select the `OpenHAK` board;
- Attempt compilation with the `Verify` Button;
- In case of issues with the strings some of the libraries may need to be edited (related to the macro F)

- run the launch file `roslaunch haptic_glove_ros haptic_glove.launch dev_path:=/dev/ttyUSB0` specifing the USB port of the glove.


Another way is to run the serial node autonomously `rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=250000`. In many cases it gets problems if it is launched from launch file.
`roslaunch haptic_glove_ros haptic_glove.launch dev_path:=/dev/ttyUSB0 baud_rate:=250000` launches the application nodes that control the vibration using the optoforce topics.
