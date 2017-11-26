#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ -f devel/setup.bash ]; then
   source "${DIR}"/devel/setup.bash
fi
#source src/grudsby_gazebo/models/moveToModelLibrary.sh
