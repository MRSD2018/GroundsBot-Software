#!/bin/bash

if [ -f devel/setup.bash ]; then
    source devel/setup.bash
fi
export GAZEBO_PLUGIN_PATH=`pwd`/devel/lib:$GAZEBO_PLUGIN_PATH
