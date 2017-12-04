# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
import rospkg

import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget
from rqt_gui_py.plugin import Plugin

from std_msgs.msg import Int16
#from counter_node.srv import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


class GrudsbyGUI(Plugin):

    def gps_msg_callback(self, msg_in):
        self._widget.LatLng_msg.setText("Lat: " + '%5f'%msg_in.latitude + " Lng: " + '%5f'%msg_in.longitude + " Alt: " + '%5f'%msg_in.altitude)
        
    def imu_msg_callback(self, msg_in):
	self._widget.IMU_msg.setText("w: " + str(msg_in.orientation.w) + " x: " + str(msg_in.orientation.x) + " y: " + str(msg_in.orientation.y) + " z: " + str(msg_in.orientation.z) + " ang_x: " + str(msg_in.angular_velocity.x) + " ang_y: " + str(msg_in.angular_velocity.y) + " ang_z: " + str(msg_in.angular_velocity.z))

    def odom_msg_callback(self, msg_in):
        self._widget.Odom_msg.setText("w: " + str(msg_in.pose.pose.orientation.w) + " x: " + str(msg_in.pose.pose.orientation.x) + " y: " + str(msg_in.pose.pose.orientation.y) + " z: " + str(msg_in.pose.pose.orientation.z) + " pos_x: " + str(msg_in.pose.pose.position.x) + " pos_y: " + str(msg_in.pose.pose.position.y) + " pos_z: " + str(msg_in.pose.pose.position.z))

    def __init__(self, context):
        super(GrudsbyGUI, self).__init__(context)
        self.setObjectName('GrudsbyGUI')
        
        self.message_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)

        rospy.Subscriber("GPS_PUB_TOPIC", NavSatFix, self.gps_msg_callback)
        rospy.Subscriber("IMU_PUB_TOPIC", Imu, self.imu_msg_callback)
        rospy.Subscriber("ENCODER_PUB_TOPIC", Odometry, self.odom_msg_callback)

        #rospy.Subscriber("reply_msg", reply_msg, self.reply_msg_callback)
        # rospy.Subscriber("arithmetic_reply", arithmetic_reply, self.arithmetic_reply_msg_callback)

        self.msg_to_send = Twist()
        self.counter_req_id = -1

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('grudsby_gui'), 'resource', 'GrudsbyGUI.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('GrudsbyGUIui')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        #self._widget.message_to_send.textChanged.connect(self._on_msg_to_send_changed)
        #self._widget.counter_val_to_get.textChanged.connect(self._on_counter_val_to_get_changed) 
        #self._widget.counter_val_to_get.setInputMask('9')   

        self._widget.forwardLeft.pressed.connect(self._on_left_button_pressed)
        self._widget.forwardLeft.released.connect(self._on_left_button_released)
        self._widget.forwardStraight.pressed.connect(self._on_forward_button_pressed)
        self._widget.forwardStraight.released.connect(self._on_forward_button_released)
        self._widget.forwardRight.pressed.connect(self._on_right_button_pressed)
        self._widget.forwardRight.released.connect(self._on_right_button_released)
        
    #def _on_msg_to_send_changed(self, msg):
    #    msg = str(msg)
    #    self.msg_to_send.message = msg
  
    def _on_left_button_pressed(self):
        self.msg_to_send.linear.x = 0.6
        self.msg_to_send.angular.z = -0.8
        self.message_pub.publish(self.msg_to_send)

    def _on_forward_button_pressed(self):
        self.msg_to_send.linear.x = 1.0
        self.msg_to_send.angular.z = 0.0
        self.message_pub.publish(self.msg_to_send)

    def _on_right_button_pressed(self):
        self.msg_to_send.linear.x = 0.6
        self.msg_to_send.angular.z = 0.8
        self.message_pub.publish(self.msg_to_send)

    def _on_left_button_released(self):
        self.msg_to_send.linear.x = 0.0
        self.msg_to_send.angular.z = 0.0
        self.message_pub.publish(self.msg_to_send)

    def _on_forward_button_released(self):
        self.msg_to_send.linear.x = 0.0
        self.msg_to_send.angular.z = 0.0
        self.message_pub.publish(self.msg_to_send)

    def _on_right_button_released(self):
        self.msg_to_send.linear.x = 0.0
        self.msg_to_send.angular.z = 0.0
        self.message_pub.publish(self.msg_to_send)


    #def _on_counter_val_to_get_changed(self, id):
    #    try:
    #        self.counter_req_id = int(id)
    #    except ValueError:
    #        print('String input is not an integer')

    # def _on_send_request_pressed(self):
    #     rospy.wait_for_service('message_counter')
    #     try:
    #         counter_serv = rospy.ServiceProxy('message_counter',counter)
    #         response = counter_serv(self.counter_req_id)
    #         self.message_count_display(response)
    #         return response
    #     except rospy.ServiceException, ex:
    #         print "Service call to get message counter failed. %s"%e


    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
        
        
        
        
        
        
        
        
