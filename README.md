# GroundsBot Software
Grudsby's Brains 

## Installation
### Prerequisites
#### ROS:
Grudsby uses ROS Kinetic, on Ubuntu 16.04.

#### If using Anaconda:
comment out the path anaconda PATH setting in ~/.bashrc
```bash
#export PATH="/home/"user"/"anaconda version"/bin:$PATH"
```
#### Arduino
Install the Arduino IDE from the Arduino site. Do not install with apt-get. 


#### Necessary Packages:
- rosserial 

```bash
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
```

-rospack
```bash
sudo apt-get install ros-kinetic-rospack
```

-rospy
```bash
sudo apt-get install ros-kinetic-rospy
```


-ros teleop-tools
```bash
sudo apt-get install ros-kinetic-teleop-tools
```

-catkin_pkg
```bash
pip install catkin_pkg
```

-rospkg 
```bash
pip install rospkg
```
-SBUS and elapsedMillis 
```
Move these from the Libraries folder to your Arduino/libraries folder
```


### Configuring Rosserial:
The custom Ros-lib Arduino package has been included in the Libraries folder.
Copy this library over to your Arduino/libraries folder.

If needed:
Making custom headers for rosserial-arduino:

Source the sourceme file in Grudsby-Software

then:
```bash 
 rm -rf ~/Arduino/libraries/ros_lib/
 rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/
```



