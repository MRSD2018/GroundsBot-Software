nmea_navsat_driver
===============

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver

RUN SOLO
---
rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyACM0 _baud:=57600
