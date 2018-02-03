### If using Anaconda:
#comment out the path anaconda PATH setting in ~/.bashrc
#export PATH="/home/"user"/"anaconda version"/bin:$PATH"

sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
sudo apt-get install ros-kinetic-rospack
sudo apt-get install ros-kinetic-rospy
sudo apt-get install ros-kinetic-teleop-twist-keyboard
sudo apt-get install ros-kinetic-uuid-msgs
sudo apt-get install libi2c-dev i2c-tools
sudo apt-get install ros-kinetic-tf2-geometry-msgs 
sudo apt-get install ros-kinetic-unique-id
sudo apt-get install ros-kinetic-hector-gazebo-plugins 

pip install catkin_pkg
pip install rospkg

#Move these from the Libraries folder to your Arduino/libraries folder

### Configuring Rosserial:
#The custom Ros-lib Arduino package has been included in the Libraries folder.
#Copy this library over to your Arduino/libraries folder.

#If needed:
#Making custom headers for rosserial-arduino:

#Source the sourceme file in Grudsby-Software

#then:
# rm -rf ~/Arduino/libraries/ros_lib/
# rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries/

### Style
#Use the ```.clang_format``` file to reformat code. 

#Packages must be run through clang-tidy: 

sudo apt-get install clang libclang-dev clang-tidy-3.8 clang-format-3.8


#run-clang-tidy-3.8.py -p ~/Groundsbot-Software/build grudsby_package 
#run-clang-tidy-3.8.py -p ~/Groundsbot-Software/build grudsby_package -fix





