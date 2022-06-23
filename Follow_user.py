#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from qt_nuitrack_app.msg import Faces
from qt_robot_interface.srv import *
from qt_motors_controller.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

#We itialize vectors that contain right and left eye coordinates
right_eye = [0.5,0.5]
left_eye = [0.5,0.5]

#Define a ros publisher which we will use to publish the new position of the head motors  
head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)

#Define a function that update the values of the vectors right_eye and left_eye
def face_callback(msg):
    global right_eye
    global left_eye
    right_eye = msg.faces[0].right_eye
    left_eye = msg.faces[0].left_eye

if __name__ == '__main__':

    #We initialize the ROS node for the process
    rospy.init_node('my_tutorial_node')

    #We indicate that the script was lauched with success
    rospy.loginfo("Now, I'm going to follow you !")

    #Initialize the vector that contains the current head_yaw and head_pitch positions   
    current_headyaw_pos = 0
    current_headpitch_pos = 0

    #Stock the last d_x detected
    last_dx = 0

    #Start an infinite loop
    while not rospy.is_shutdown():

        #Wait until the robot is in its new position
        rospy.sleep(1)

        try:

            #Define a ros subscriber which will call each time face_callback function  
            rospy.Subscriber('/qt_nuitrack_app/faces', Faces, face_callback)

            #We define a new Float64MultiArray message for the head motors           
            href = Float64MultiArray()

            #We calculate the coordinates of a point located between the two eyes that the robot will follow
            new_x_face = (right_eye[0]+left_eye[0])/2
            new_y_face = (right_eye[1]+left_eye[1])/2

            #We calculate distance between the new face position detected and the point with coordinates (0.5,0.5)            
            d_x = 0.5-new_x_face
            d_y = new_y_face-0.5

            #If d_x keeps the same value, this means that there's no new detection
            if (d_x == last_dx):
                new_detection = 0
                next_headyaw_pos = 0
                
            else: 
                new_detection = 1

            #Calculate the distance
            d = (d_x**2+d_y**2)**0.5

            #Start with a face looking forward
            next_headpitch_pos = 0

            #Make changes if there's a new detection and the distance d is higher than 0.15
            if d>0.15 and new_detection == 1:
                new_degre = np.arcsin(d_x/np.sqrt(0.5269**2+d_x**2))*180/np.pi
                next_headyaw_pos = current_headyaw_pos + new_degre
                if d_y <0:
                    next_headpitch_pos = np.arcsin(d_y*0.964/(d_y**2+0.92**2-2*d_y*0.92*0.267)**0.5)*180/np.pi + current_headpitch_pos
                else:
                    next_headpitch_pos = np.arcsin(d_y*0.964/(d_y**2+0.92**2-2*d_y*0.92*-0.267)**0.5)*180/np.pi + current_headpitch_pos                

            #Respect the motors' limits
            if next_headyaw_pos > 60 : next_headyaw_pos = 60
            if next_headyaw_pos < -60 : next_headyaw_pos =-60
            if next_headpitch_pos > 10 : next_headpitch_pos = 10
            if next_headpitch_pos < -10 : next_headpitch_pos =-10   
                   
            #We publish the robot's head_yaw and head_pitch position 
            href.data = [next_headyaw_pos,next_headpitch_pos+3]
            head_pub.publish(href)
            head_pub.publish(href)
            
            #See if the robot detects a face
            print (d, new_detection)
            
            #We update value of the current distance and positions
            current_headyaw_pos = next_headyaw_pos   
            current_headpitch_pos= next_headpitch_pos
            last_dx = d_x

            #We add a sleep time
            rospy.sleep(1)

        #Continue if the user write something on the keyboard   
        except KeyboardInterrupt:
            pass

    #Show to the user when the script stops running
    rospy.loginfo("I won't follow you anymore !")
    

