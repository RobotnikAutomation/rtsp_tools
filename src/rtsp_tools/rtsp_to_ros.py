#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python
import cv2

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import *

from rcomponent.rcomponent import *

class RTSPToRos(RComponent):

    def __init__(self):
        # Init default values
        self.resource = ""
        self.camera_name = ""
        self.camera_frame = ""

        self.cap = None
        self.ros_cv_bridge = None

        self.camera_info_msg = CameraInfo()

        RComponent.__init__(self)

    def ros_read_params(self):
        """Gets params from param server"""
        RComponent.ros_read_params(self)

        self.resource = rospy.get_param('~rtsp_resource', self.resource)
        self.camera_name = rospy.get_param('~camera_name', self.camera_name)
        self.camera_frame = rospy.get_param('~camera_frame', self.camera_frame)


    def ros_setup(self):
        """Creates and inits ROS components"""

        self.image_pub = rospy.Publisher("~image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)

        RComponent.ros_setup(self)
        return 0
    
    def setup(self):
        self.cap = cv2.VideoCapture(self.resource)

        if not self.cap.isOpened():
            rospy.logerr("%s::setup: Error opening resource `%s`. Please check." % \
                (self._node_name, self.resource))
            exit(0)
        
        rospy.loginfo("%s::setup: Resource (%s) successfully opened" % \
            (self._node_name, self.resource))

        # initialize ROS_CV_Bridge
        self.ros_cv_bridge = CvBridge()

        # initialize Camera Info Manager
        self.camera_info_manager = CameraInfoManager(cname=self.camera_name, \
            namespace=self.camera_name)
        self.camera_info_manager.loadCameraInfo()
        if not self.camera_info_manager.isCalibrated():
            rospy.logwarn("%s::setup: No calibration found for the current camera" \
                % self._node_name)


        return RComponent.setup(self)

    def ready_state(self):
        """Actions performed in ready state"""

        # get new frame
        rval, cv_image = self.cap.read()
        # handle Ctrl-C
        #key = cv2.waitKey(20)
        #if rospy.is_shutdown() or key == 27 or key == 1048603:
        #    break
        # convert CV image to ROS message
        image_msg = self.ros_cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_msg.header.frame_id = self.camera_frame
        image_msg.header.stamp = rospy.Time.now()
        self.image_pub.publish( image_msg )

        return

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    def ros_publish(self):
        self.update_camera_info_msg()
        self.camera_info_pub.publish( self.camera_info_msg )

        RComponent.ros_publish(self)

    def update_camera_info_msg(self):
        self.camera_info_msg = self.camera_info_manager.getCameraInfo()
        self.camera_info_msg.header.frame_id = self.camera_frame
        self.camera_info_msg.header.stamp = rospy.Time.now()
