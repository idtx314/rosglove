#!/usr/bin/env python


#Receive messages from glove_transform
#Translate message into tf frame and broadcast


import rospy
import tf
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose



pose = Pose()


#Updates pose whenever new data is received
def callback(data):
    global pose

    #Translate into radians
    data.x = data.x * 3.1415/180.
    data.y = data.y * 3.1415/180.
    data.z = data.z * 3.1415/180.

    #Convert to a quaternion
    var = tf.transformations.quaternion_from_euler(data.x, data.y, data.z)

    #Update pose
    pose.orientation.x = var[0]
    pose.orientation.y = var[1]
    pose.orientation.z = var[2]
    pose.orientation.w = var[3]





def main():

    #Init
    rospy.init_node('transform_broadcaster')
    rospy.Subscriber("glove_transform", Point, callback)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)  #Publish rate

    global pose
    pose.orientation.w = 1;

    #Broadcast frames
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                         rospy.Time.now(),
                         "glove_frame",
                         "world_frame")
        rate.sleep()













if __name__ == '__main__':
    main()
