#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from qt_nuitrack_app.msg import Faces
from qt_robot_interface.srv import *
from qt_motors_controller.srv import *
from qt_gesture_controller.srv import gesture_play
from qt_nuitrack_app.msg import Gestures
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from qt_nuitrack_app.msg import Skeletons
import numpy as np
import random


HEAD_YAW_RANGE = 15         #Range of head yaw for free movement 
HEAD_PITCH_RANGE = 7        #Range of head pitch for free movement  
SHOULDER_PITCH = 10         #Range of shoulder pitch
SHOULDER_ROLL = 5           #Range of shoulder roll
VELOCITY = 2                #Velocity of free movement
DISTANCE = 0.15             #Threshold of follow
SENSITIVITY_NAMASTE = 0.002 #Threshold of namaste


#We itialize vectors that contain right and left eye coordinates and a parameter to make sure that the robot won't make the "Namasté" two times in a row
right_eye = (0.5,0.5)
left_eye = (0.5,0.5)
left_hand = [1,0]
right_hand = [0,0]
collar = [0,0]
i=0

#Define a function that update the values of the vectors collar, right hand and left_eye
def skeleton_callback(msg):
    
    global left_hand,collar,right_hand,i
    skeleton = msg.skeletons[-1]
    
    #Take coordinates of collar, left hand and right hand. 
    collar = skeleton.joints[5].projection      #The 5 is the number of joint collar in Nuitrack Skeleton
    left_hand = skeleton.joints[9].projection   #The 9 is the number of joint left hand in Nuitrack Skeleton
    right_hand = skeleton.joints[15].projection #The 15 is the number of joint right hand in Nuitrack Skeleton

    #We calculate left and right eye coordinates after choosing collar as reference 
    x_left = left_hand[0]-collar[0] 
    y_left = left_hand[1]-collar[1]
    x_right = right_hand[0]-collar[0] 
    y_right = right_hand[1]-collar[1]
    d = x_left**2+y_left**2+x_right**2+y_right**2

    #The robot says and does "Namasté" only once when the distance d is lower then 0.002
    if d<SENSITIVITY_NAMASTE and i==0:
        behaviorTalkText_pub.publish("Namasté!")
        gesturePlay("my_hello",0)
        i=1

    #When d increases and takes a value higher than 0.1, i becomes null. The robot can do "Namasté" again if d takes a value < 0.002  
    if d>=0.1: i=0     

def face_callback(msg):
    global right_eye
    global left_eye
    right_eye = msg.faces[0].right_eye
    left_eye = msg.faces[0].left_eye
    
    #speech_pub.publish("Right eye")
        
    #rospy.sleep(2)

    #setVelocity = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)  
    #rospy.wait_for_service('/qt_robot/motors/setVelocity')
def gesture_callback(msg):
    if msg.gestures[0].name == "SWIPE UP" or msg.gestures[0].name == "WAVING":
        behaviorTalkText_pub.publish("Bonjour !")
        gesturePlay("QT/up_left",0)

#When calling this function, the robot does random moves (head included) when i=1 else, it does random hand moves when i=1    
def random_mvt(i):

    RightShoulderPitch_ref = random.randrange(-90-SHOULDER_PITCH,-90+SHOULDER_PITCH)
    RightShoulderRoll_ref = random.randrange(-70-SHOULDER_ROLL,-70+SHOULDER_ROLL)
    LeftShoulderPitch_ref = random.randrange(-90-SHOULDER_PITCH,-90+SHOULDER_PITCH)
    LeftShoulderRoll_ref = random.randrange(-70-SHOULDER_ROLL,-70+SHOULDER_ROLL)
    href = Float64MultiArray()
    l_ref = Float64MultiArray()
    r_ref = Float64MultiArray()
    #Generate a random angle for head. 
    if i==1:
        head_yaw_ref = random.randrange(-HEAD_YAW_RANGE,HEAD_YAW_RANGE)
        head_pitch_ref = random.randrange(-HEAD_PITCH_RANGE,HEAD_PITCH_RANGE)    
        href.data = [head_yaw_ref, head_pitch_ref] 
        head_pub.publish(href) 
    r_ref.data = [RightShoulderPitch_ref, RightShoulderRoll_ref,-20]
    l_ref.data = [LeftShoulderPitch_ref, LeftShoulderRoll_ref,-20]
    
    right_pub.publish(r_ref)
    left_pub.publish(l_ref)
    setVelocity(['left_arm', 'right_arm', 'HeadPitch'],VELOCITY)
    
if __name__ == '__main__':
    
    #We initialize the ROS node for the process
    rospy.init_node('my_tutorial_node')

    #We indicate that the script was lauched with success
    rospy.loginfo("I can follow you, and I can also say hi to you !")

    #Initialize the vector that contains the current head_yaw and head_pitch positions   
    current_headyaw_pos = 0
    current_headpitch_pos = 0

    #Stock the last d_x detected
    last_dx = 0

    #Define Subscribers, Publisher and Services that we'll need
    rospy.Subscriber('/qt_nuitrack_app/faces', Faces, face_callback)
    rospy.Subscriber('/qt_nuitrack_app/gestures', Gestures, gesture_callback)
    behaviorTalkText_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    rospy.Subscriber('/qt_nuitrack_app/skeletons', Skeletons, skeleton_callback)
    setVelocity = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)
    head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)
    right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=10)
    left_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=10)


    while not rospy.is_shutdown():
    
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
            if d>DISTANCE and new_detection == 1:
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

            
            #The robot makes random hand moves
            random_mvt(0)
            
            #We add a sleep time
            rospy.sleep(0.8)
            

        except KeyboardInterrupt:
            pass
        
    #Show to the user when the script stops running
    rospy.loginfo("finished!")
    

    
    

