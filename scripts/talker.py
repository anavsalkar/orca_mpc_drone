#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from numpy.lib.function_base import vectorize
import rospy
from std_msgs.msg import String
import numpy as np
import std_msgs.msg
import math
#import tf
from geometry_msgs.msg import Transform, Quaternion, Point, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
import time
sys.path.append('../../')
import do_mpc

#from template_model import template_model


def talker():
    pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        # accl_z = 1
        # msg = MultiDOFJointTrajectory()
        # header = std_msgs.msg.Header()
        # header.stamp = rospy.Time()
        # header.frame_id = '1'
        # msg.joint_names.append('base')
        # msg.header = header
        

        # # create start point for trajectory
        # transforms = Transform()
        # velocities = Twist()
        # accel = Twist()
        # point = MultiDOFJointTrajectoryPoint()
        # msg.points.append(point)

        # # create end point for trajectory
        # transforms.translation.x = 0
        # transforms.translation.y = 0
        # transforms.translation.z = 15

        # # quat = tf.transformations.quaternion_from_euler(yaw*np.pi/180.0, 0, 0, axes = 'rzyx')
        # # transforms.rotation.x = quat[0]
        # # transforms.rotation.y = quat[1]
        # # transforms.rotation.z = quat[2]
        # # transforms.rotation.w = quat[3]

        # # velocities.linear.x = 1*math.cos(0.2*rospy.get_time())
        # # velocities.linear.y = -1*math.sin(0.2*rospy.get_time())
        # # velocities.linear.z = 0
        
        # # velocities.angular.x = wx
        # # velocities.angular.y = wy
        # # velocities.angular.z = wz
        
        # # accel.linear.x = 0
        # # accel.linear.y = 0
        # # accel.linear.z = 0
        
        # # accel.angular.x = 0
        # # accel.angular.y = 0
        # # accel.angular.z = 0
        
        # #point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time())
        # point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.get_rostime())
        # #msg.points.append(point)

        # #rospy.sleep(1)
        # #pub.publish(msg)
        
        
        
        
        # rospy.loginfo(hello_str)
        # pub.publish(msg)
        # rate.sleep()
        
        firefly_command_publisher = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

        desired_yaw_to_go_degree=-10

        desired_x_to_go=0
        desired_y_to_go=0
        desired_z_to_go=0

        #quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(desired_yaw_to_go_degree))

        traj = MultiDOFJointTrajectory()

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()
        header.frame_id = 'frame'
        traj.joint_names.append('base_link')
        traj.header=header

        transforms =Transform(translation=Point(desired_x_to_go, desired_y_to_go, desired_z_to_go), rotation=Quaternion())

        velocities =Twist()
        accelerations=Twist()
        
        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = 0.1
        
        accelerations.angular.x = 0
        accelerations.angular.y = 0
        accelerations.angular.z = 0
        
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())

        traj.points.append(point)

        rate.sleep()
        firefly_command_publisher.publish(traj)
        print('starting')
        
        
        
        
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
