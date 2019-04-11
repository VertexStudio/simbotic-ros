#!/usr/bin/env python

# Publishes `poseStamped and pointStamped`.

import rospy
from math import sqrt
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image
from osc4py3.as_eventloop import (osc_startup, osc_udp_server, osc_method,
                                  osc_process, osc_terminate)
from osc4py3 import oscmethod as osm


def handleOSCPoseMessage(px, py, pz, qx, qy, qz, qw):
    agentPose = PoseStamped()

    agentPose.header.stamp = rospy.Time.now()
    agentPose.header.frame_id = "unreal"
    agentPose.pose.position.x = float(px)/100
    agentPose.pose.position.y = -float(py)/100
    agentPose.pose.position.z = float(pz)/100
    quatX = float(qx)/100
    quatY = float(qy)/100
    quatZ = float(qz)/100
    quatW = float(qw)/100
    quatLen = sqrt(quatX**2 + quatY**2 + quatZ**2 + quatW**2)

    # Normalized quaternion
    agentPose.pose.orientation.x = quatX/quatLen
    agentPose.pose.orientation.y = quatY/quatLen
    agentPose.pose.orientation.z = -quatZ/quatLen
    agentPose.pose.orientation.w = quatW/quatLen

    poseStr = "Position(%s, %s, %s), Quaternion(%s, %s, %s, %s)" % (
        agentPose.pose.position.x, agentPose.pose.position.x, agentPose.pose.position.x, 
        agentPose.pose.orientation.x, agentPose.pose.orientation.y, agentPose.pose.orientation.z, agentPose.pose.orientation.w)
    rospy.loginfo("PoseStamped")
    rospy.loginfo(poseStr)

    pubPose.publish(agentPose)
    rate.sleep()

def handleOSCPointMessage(px, py, pz):
    agentPoint = PointStamped()

    agentPoint.header.stamp = rospy.Time.now()
    agentPoint.header.frame_id = "unreal"
    agentPoint.point.x = float(px)/100
    agentPoint.point.y = -float(py)/100
    agentPoint.point.z = float(pz)/100

    pointStr = "Position(%s, %s, %s)" % (
        agentPoint.point.x, agentPoint.point.x, agentPoint.point.x)
    rospy.loginfo("PointStamped")
    rospy.loginfo(pointStr)

    pubPoint.publish(agentPoint)
    rate.sleep()


def handleOSCImageMessage(data):
    agentImage = Image()
    
    agentImage.header.stamp = rospy.Time.now()
    agentImage.header.frame_id = "unreal"
    agentImage.encoding = "rgba8"
    agentImage.height = 30
    agentImage.width = 30
    agentImage.data = data.tobytes()
    agentImage.is_bigendian = 0
    agentImage.step = 30 * 4

    rospy.loginfo("Publishing...")
    pubImage.publish(agentImage)
    rate.sleep()

osc_startup()
osc_udp_server("0.0.0.0", 7000, "UnrealGAMS")

osc_method("/agent/0/pose", handleOSCPoseMessage)
osc_method("/agent/0/pos", handleOSCPointMessage)
osc_method("/agent/0/rgb", handleOSCImageMessage)

pubPose = rospy.Publisher('agent/pose', PoseStamped, queue_size=10)
pubPoint = rospy.Publisher('agent/point', PointStamped, queue_size=10)
pubImage = rospy.Publisher('agent/rgb', Image, queue_size=10)

rospy.init_node('agent', anonymous=True)
rate = rospy.Rate(100)  # 100hz

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            osc_process()
        osc_terminate()
    except rospy.ROSInterruptException:
        pass
