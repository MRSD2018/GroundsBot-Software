cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_CXX_STANDARD 11)
project( boustrophedon )

include_directories(include)
set ( SOURCES src/boustrophedon.cpp)
add_executable(boustrophedon ${SOURCES})


