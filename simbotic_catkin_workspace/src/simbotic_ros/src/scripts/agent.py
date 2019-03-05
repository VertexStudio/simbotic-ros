#!/usr/bin/env python

# Publishes `pose` of agent to the `/agent/pose` topic.

import rospy
from math import sqrt
from geometry_msgs.msg import PoseStamped, Quaternion
from osc4py3.as_eventloop import (osc_startup, osc_udp_server, osc_method,
                                  osc_process, osc_terminate)
from osc4py3 import oscmethod as osm


def handleOSCPoseMessage(px, py, pz, qx, qy, qz, qw):
    agentP = PoseStamped()

    agentP.header.stamp = rospy.Time.now()
    agentP.header.frame_id = "unreal"
    agentP.pose.position.x = float(px)/100
    agentP.pose.position.y = -float(py)/100
    agentP.pose.position.z = float(pz)/100
    quatX = float(qx)/100
    quatY = float(qy)/100
    quatZ = float(qz)/100
    quatW = float(qw)/100
    quatLen = sqrt(quatX**2 + quatY**2 + quatZ**2 + quatW**2)

    # Normalized quaternion
    agentP.pose.orientation.x = quatX/quatLen
    agentP.pose.orientation.y = quatY/quatLen
    agentP.pose.orientation.z = -quatZ/quatLen
    agentP.pose.orientation.w = quatW/quatLen

    poseStr = "Position(%s, %s, %s), Quaternion(%s, %s, %s, %s)" % (
        agentP.pose.position.x, agentP.pose.position.x, agentP.pose.position.x, 
        agentP.pose.orientation.x, agentP.pose.orientation.y, agentP.pose.orientation.z, agentP.pose.orientation.w)
    rospy.loginfo(poseStr)

    pub.publish(agentP)
    rate.sleep()

osc_startup()
osc_udp_server("0.0.0.0", 7000, "UnrealGAMS")

osc_method("/agent/0/pose", handleOSCPoseMessage)

pub = rospy.Publisher('agent/pose', PoseStamped, queue_size=10)
rospy.init_node('agent', anonymous=True)
rate = rospy.Rate(10)  # 10hz

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            osc_process()
        osc_terminate()
    except rospy.ROSInterruptException:
        pass
