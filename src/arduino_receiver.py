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
    pub2 = rospy.Publisher("glove_transform", Point, queue_size=10)
    rospy.init_node('data_pub')
    #rate = rospy.Rate(50)  #Publish rate should be controlled by arduino

    #Init Variables
    arr = []
    data = []
    s = ""
    flag = 0
    msg = Point()
    euler = Point()


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
                        msg.x = data[0]     #Mag x
                        msg.y = data[1]     #Mag y
                        msg.z = data[2]     #Mag z
                        #magnitude = data[3];
                        euler.x = data[4]   #Roll
                        euler.y = data[5]   #Pitch
                        euler.z = data[6]   #Yaw
                        pub.publish(msg)
                        pub2.publish(euler)
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
            sleep(.25)            #Wait .25 second

if __name__ == '__main__':
    main()
