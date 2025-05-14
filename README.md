# com3528_2025_team9
This Code works with the physical MiRO-E be sure to change the name of the robot to match robot used in actions.launch and command_handler.launch

Package dependancies run all these commands before use(sudo privilages are required):
```
Sudo apt install portaudio19-dev libasound-dev libsndfile1-dev
```
```
Pip install SpeechRecognition
```
```
Pip install pyaudio
```
```
Sudo apt install python3-pyaudio
```

To use this package install all dependencies and connect to the robot, changing the robot name value to fit. 

Then do: 
```
roslaunch com3528_2025_team9 command_handler.launch
```

***

Speech to text package: install and instructions can be found here: https://github.com/EmaroLab/ros_verbal_interaction_node

Dog sound effect: https://pixabay.com/sound-effects/small-dog-barking-84707/

Beep sound effect: https://pixabay.com/sound-effects/beep-warning-6387/

Licence information for SFX: https://pixabay.com/service/license-summary/
