#!/usr/bin/env python

#Program Spec: Initiate a node, subscribe to /glove data, whenever I receive a point from glove data, substitute it for the marker end point and publish the marker to /visualization_marker

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


msg = Marker()
start = Point()
end = Point()


def callback(data):
    global end

    end.x = data.x/100.0
    end.y = data.y/100.0
    end.z = data.z/100.0



def main():
    #init publisher
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.init_node('glove_publisher')
    rospy.Subscriber("glove_data", Point, callback)
    rate = rospy.Rate(100)  #Publish rate

    global msg
    global start
    global end

    #prep message
    msg.header.frame_id = "magnetometer_frame"
    msg.ns = "space"
    ''' #Set by default
    msg.id = 0
    msg.type = Marker.ARROW
    msg.action = Marker.ADD
    '''
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.1   #Shaft radius
    msg.scale.y = 0.5   #head radius
    msg.scale.z = 0.5   #head length
    msg.color.a = 1.0
    msg.color.g = 1.0

    start.x = start.y = start.z = 0.0
    end.x = end.y = end.z = 2.0

    #Assign the memory addresses, not just the values, to the message
    msg.points.append(start)
    msg.points.append(end)


    #loop
    while not rospy.is_shutdown():
        #Stamp Message
        msg.header.stamp = rospy.Time.now()

        #publish message
        pub.publish(msg)
        rate.sleep()












if __name__ == '__main__':
    main()
