#!/usr/bin/env python
import sys
import rospy
from qt_gesture_controller.srv import gesture_play
from qt_nuitrack_app.msg import Gestures
from std_msgs.msg import String
from qt_robot_interface.srv import *
from qt_nuitrack_app.msg import Skeletons

#We itialize vectors that contain right and left eye coordinates and a parameter to make sure that the robot won't make the "Namasté" two times in a row
left_hand = [1,0]
right_hand = [0,0]
collar = [0,0]
i=0

#Define a function that update the values of the vectors collar, right hand and left_eye
def skeleton_callback(msg):
    
    global left_hand,collar,right_hand,i
    skeleton = msg.skeletons[-1]
    #Take coordinates of collar, left hand and right hand. 
    collar = skeleton.joints[5].projection        #The 5 is the number of joint collar in Nuitrack Skeleton
    left_hand = skeleton.joints[9].projection     #The 9 is the number of joint left hand in Nuitrack Skeleton
    right_hand = skeleton.joints[15].projection   #The 15 is the number of joint right hand in Nuitrack Skeleton

    #We calculate left and right eye coordinates after choosing collar as reference 
    x_left = left_hand[0]-collar[0] 
    y_left = left_hand[1]-collar[1]
    x_right = right_hand[0]-collar[0] 
    y_right = right_hand[1]-collar[1]
    d = x_left**2+y_left**2+x_right**2+y_right**2

    #The robot says and does "Namasté" only once when the distance d is lower then 0.002
    if d<0.002 and i==0:
        behaviorTalkText_pub.publish("Namasté!")
        gesturePlay("my_hello",0)
        i=1

        
    #When d increases and takes a value higher than 0.1, i becomes null. The robot can do "Namasté" again if d takes a value < 0.002  
    if d>=0.1: i=0     
     
if __name__ == '__main__':

    #We initialize the ROS node for the process
    rospy.init_node('my_tutorial_node')
    
    #We indicate that the script was lauched with success    
    rospy.loginfo("Do Namasté !")
        
    #We define a ros ServiceProxy which will be used to play gestures
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)

    #We define a ros Publisher which will be used to say "Namasté"
    behaviorTalkText_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)

    #We define a ros Subscriber which will be used to detect the skeleton's joints values
    rospy.Subscriber('/qt_nuitrack_app/skeletons', Skeletons, skeleton_callback)

    try:

        #The code is repeated until the user taps ctrl+C    
        rospy.spin()       
    
        
    except (KeyboardInterrupt, ValueError) :
        pass

    #Show to the user when the script stops running
    rospy.loginfo("I won't do Namasté anymore")
        
        
 
