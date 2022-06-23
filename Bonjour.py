#!/usr/bin/env python
import sys
import rospy
from qt_gesture_controller.srv import gesture_play
from qt_nuitrack_app.msg import Gestures
from std_msgs.msg import String
from qt_robot_interface.srv import *
import random

#Define a function that make the robot say "Bonjour" and do a greeting with a smiling face when it sees "SWIPE UP"/"WAVING"
def gesture_callback(msg):
    if msg.gestures[0].name == "SWIPE UP" or msg.gestures[0].name == "WAVING":
        if random.randrange(0,2)==1:
            behaviorTalkText_pub.publish("Bonjour !")
            gesturePlay("QT/up_left",0)
            emotionShow("QT/showing_smile")
        else :
            behaviorTalkText_pub.publish("Bonjour !")
            gesturePlay("QT/up_right",0)
            emotionShow("QT/showing_smile") 
            
#We initialize the ROS node for the process
rospy.init_node('my_tutorial_node')

#We indicate when the code starts running
rospy.loginfo("Let's exchange greetings")   

if __name__ == '__main__':

    #Define a ros subscriber which will call the gesture_callback each time
    rospy.Subscriber('/qt_nuitrack_app/gestures', Gestures, gesture_callback)

    #Define a ros publisher which we will use to publish "Bonjour !"  
    behaviorTalkText_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)

    #Define a ros service which will call play the gesture when detected
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)

    #Define a ros service which will do the smiling face
    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
 
    try:
        
        #The code is repeated until the user taps ctrl+C 
        rospy.spin()
                
    except KeyboardInterrupt:
        pass

    #Show to the user when the script stops running
    rospy.loginfo("finished!")
