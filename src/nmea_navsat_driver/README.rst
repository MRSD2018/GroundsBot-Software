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

EMLID REACH Configuration
---

GrudsRover Emlid Reach unit:
  Hardware setup:
    Uart/Power to GroundsBot integration PCB
    USB OTG (USB micro <-> micro) to 3DR transceiver
  Software setup:
  (Connect to same network as reach, go to reachview app at reach.local)
    Position output:
      Serial / UART NMEA (57600 baud)
    Correction input:
      USB OTG (57600 ?)
    Base station:
      off
    
GrudsBase Emlid Reach unit:
  Hardware Setup:
    5V custom df13 cable (gnd and 5V) to 5V regulator from 12V battery
    USB OTG (USB micro <-> micro) to 3DR transceiver
  Software setup:
  (Connect to same network as reach, go to reachview app at reach.local)
    GrudsBase Correction output:
      USB OTG to 3dr transceiver
    Correction input:
     Off
    Position output:
     Off
    Base station:
      USB otg out (57600 baud, not sure it actually uses this)
