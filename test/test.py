#!/usr/bin/env python

'''
Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Karrenbauer Oliver, Meissner Pascal, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import roslib
import rospy
import json
import random
import shutil
import csv
import time
import tf
import numpy

from actionlib import *
from actionlib.msg import *
import sensor_msgs.msg
import math
import signal
import subprocess
from direct_search_manager.msg import *
import geometry_msgs.msg

if __name__ == '__main__':
    # register closing callback
    try:
        rospy.init_node('test_direct_search_manager')
        remainingRobotPoses = 10
        while remainingRobotPoses > 0 :
            rospy.loginfo('GetGoalCameraPose action server call')
            client = actionlib.SimpleActionClient('direct_search_manager', direct_search_manager.msg.directSearchAction)
            client.wait_for_server()
            goal = direct_search_manager.msg.gridGoal(command="GetGoalCameraPose", searchedObjectTypes=["Smacks"])
            client.send_goal(goal)
            client.wait_for_result()
            result = client.get_result()
            remainingRobotPoses = result.remainingRobotPoses
            quaternion = (
                result.goalRobotPose.orientation.x,
                result.goalRobotPose.orientation.y,
                result.goalRobotPose.orientation.z,
                result.goalRobotPose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
            yawDeg = euler[2]*57.2958
            rospy.loginfo("got orienation " +  str(yawDeg) + "for pose (" + str(result.goalRobotPose.position) + ")")
    except rospy.ROSInterruptException: pass

