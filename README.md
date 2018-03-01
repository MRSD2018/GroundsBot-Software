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


-ros teleop-twist-keyboard
```bash
sudo apt-get install ros-kinetic-teleop-twist-keyboard
```

-uuid-msgs
```bash
sudo apt-get install ros-kinetic-uuid-msgs
```

-i2c tools 
```bash
sudo apt-get install libi2c-dev i2c-tools
```

-tf2 geometry msgs
```bash
sudo apt-get install ros-kinetic-tf2-geometry-msgs 
```

-unique-id
```bash
sudo apt-get install ros-kinetic-unique-id
```

-hector-gazebo-plugins
```bash
sudo apt-get install ros-kinetic-hector-gazebo-plugins 
```

-grudsby_sweeping package
```bash
sudo apt install libcurl4-openssl-dev
```

-grudsby_navStack package
```bash
sudo apt install ros-kinetic-move-base
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

### Possible Build Errors:
Camera1394stereo has a build order issue. If it doesn't build due to a missing .msg module, build the driver_base package first. 
```
catkin_make --pkg driver_base
```


### Symlinking Gazebo Models:
The Gazebo models must be symlinked into the default ~/.gazebo/models folder for gazebo to find them. 

Given that the Groundsbot-Software folder is ~/Groundsbot-Software: 

```bash

ln -s -d ~/Groundsbot-Software/src/grudsby_gazebo/models/grudsby_new/ ~/.gazebo/models/grudsby_new/
ln -s -d ~/Groundsbot-Software/src/grudsby_gazebo/models/grudsby_caster/ ~/.gazebo/models/grudsby_caster/
ln -s -d ~/Groundsbot-Software/src/grudsby_gazebo/models/grudsby_imu/ ~/.gazebo/models/grudsby_imu/
ln -s -d ~/Groundsbot-Software/src/grudsby_gazebo/models/grudsby_gps/ ~/.gazebo/models/grudsby_gps/
ln -s -d ~/Groundsbot-Software/src/grudsby_gazebo/models/grass/ ~/.gazebo/models/grass/
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

### Style
Use the ```.clang_format``` file to reformat code. 

Packages must be run through clang-tidy: 

- Install clang packages: 
```bash
sudo apt-get install clang libclang-dev clang-tidy-3.8 clang-format-3.8
```

- Add ```set(CMAKE_EXPORT_COMPILE_COMMANDS ON)``` to every package's CMakeLists.txt and re-run catkin_make

- Check code with 
```bash
 run-clang-tidy-3.8.py -p ~/Groundsbot-Software/build grudsby_package 
```
- You can try the autofixer, but be sure to check it after
```bash
 run-clang-tidy-3.8.py -p ~/Groundsbot-Software/build grudsby_package -fix
```





