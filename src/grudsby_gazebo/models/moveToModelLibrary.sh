cp -r grudsby ~/.gazebo/models
cp -r grudsby_caster ~/.gazebo/models
cp -r grudsby_new ~/.gazebo/models

export GAZEBO_PLUGIN_PATH=`pwd`/devel/lib:$GAZEBO_PLUGIN_PATH
