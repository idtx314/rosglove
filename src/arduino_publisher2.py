#!/usr/bin/env python

#Program Spec: Initiate a node, subscribe to /glove data, whenever I receive a point from glove data, substitute it for the marker end point and publish the marker to /visualization_marker

#Maybe just have the non class version update the point whenever callback and publish constantly?

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point





class processor:

    def __init__(self):
        self.pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        #Does it make sense to declare the subscriber like this? It isn'a variable in example code. Maybe it's local?
        self.sub = rospy.Subscriber("glove_data", Point, self.callback)

        #Prep Message
        self.msg = Marker()
        self.msg.header.frame_id = "base_link"
        self.msg.ns = "space"
        ''' #Set by default
        self.msg.id = 0
        self.msg.type = Marker.ARROW
        self.msg.action = Marker.ADD
        '''
        self.msg.pose.orientation.w = 1.0
        self.msg.scale.x = 0.1  #Shaft radius
        self.msg.scale.y = 0.5   #head radius
        self.msg.scale.z = 0.5   #head length
        self.msg.color.a = 1.0
        self.msg.color.g = 1.0

        #Configure a point and add it to the marker
        self.start = Point()
        self.end   = Point()
        self.start.x = self.start.y = self.start.z = 0.0
        self.end.x   = self.end.y   = self.end.z   = 2.0    #Placeholder
        #Assign the memory addresses, not just the values, to the message. The message will be updated if the point is updated.
        self.msg.points.append(start)
        self.msg.points.append(end)

    def callback(self, data):
        #data is a point Message containing the x, y, and z magnitude of the magnetometer vector. Whenever a new point is received, substitute it for the endpoint in the marker message and publish the message.
        print data
        self.end.x = data.x
        self.end.y = data.y
        self.end.z = data.z

        #Set Time stamp
        self.msg.header.stamp = rospy.Time.now()

        self.pub.publish(self.msg)
        # rate.sleep()



def main():
    #init node
    rospy.init_node('glove_publisher')
    #rate = rospy.Rate(50)  #Publish rate, not sure how this works with class structure


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")




if __name__ == '__main__':
    main()
