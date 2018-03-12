#!/usr/bin/env python

import serial
import string
import rospy
from time import sleep
from geometry_msgs.msg import Point


def main():

    #Init serial
    ser = serial.Serial('/dev/ttyUSB0',9600)

    #Init publisher
    pub = rospy.Publisher("glove_data", Point, queue_size=10)
    rospy.init_node('data_pub')
    #rate = rospy.Rate(50)  #Publish rate should be controlled by arduino

    #Init Variables
    arr = []
    data = []
    s = ""
    flag = 0
    msg = Point()


    #Look for a message
    while not rospy.is_shutdown():      #It will shut down the nodes but continue the loop if you don't check for is_shutdown()

        if (ser.in_waiting):
            c = ser.read()
            if (c == '\n'):
                    #Convert array to string
                    s = ''.join(arr)

                    #Split string into array
                    data = string.split(s,',')

                    if (flag > 14):
                        #Convert elements of the array to floats
                        for x in range(len(data)):
                            data[x] = float(data[x])

                        #Export data
                        msg.x = data[0]
                        msg.y = data[1]
                        msg.z = data[2]
                        #mag = data[3]
                        pub.publish(msg)
                        # rate.sleep()
                        print data
                    else:
                        print s
                        flag+=1

                    #Recalibrate
                    arr = []
                    data = []
                    s = ""
            else:
                arr.append(c)   #Append c to array


        else:
            sleep(1)            #Wait 1 second

'''
uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11
uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Vector3 scale
  float64 x
  float64 y
  float64 z
std_msgs/ColorRGBA color
  float32 r
  float32 g
  float32 b
  float32 a
duration lifetime
bool frame_locked
geometry_msgs/Point[] points
  float64 x
  float64 y
  float64 z
std_msgs/ColorRGBA[] colors
  float32 r
  float32 g
  float32 b
  float32 a
string text
string mesh_resource
bool mesh_use_embedded_materials
'''
if __name__ == '__main__':
    main()
