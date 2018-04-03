cp -r grudsby ~/.gazebo/models
cp -r grass ~/.gazebo/models
cp -r grudsby_caster ~/.gazebo/models
cp -r grudsby_new ~/.gazebo/models
cp -r grudsby_gps ~/.gazebo/models
cp -r grudsby_imu ~/.gazebo/models
cp -r grudsby_laser ~/.gazebo/models

export GAZEBO_PLUGIN_PATH=`pwd`/devel/lib:$GAZEBO_PLUGIN_PATH
