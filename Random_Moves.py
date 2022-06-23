#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import random
import numpy as np
from qt_motors_controller.srv import *

#We initialize the ROS node for the process
rospy.init_node('qt_motors_command')

#We indicate that the script was lauched with success
rospy.loginfo("This is when random movements start!")

#Define a ros publisher which we will use to publish the new positions of the head motors, right hand motors and left hand motors respectively   
head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)
right_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=10)
left_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=10)

#Define a ros service which we will use to controll the velocity of the motors
setVelocity = rospy.ServiceProxy('/qt_robot/motors/setVelocity', set_velocity)  

if __name__ == '__main__':

    #Define an infinite loop
    while not rospy.is_shutdown():

        #Choose random values ​​for the positions of the motors while respecting the constraints on the limit values ​​of the motors        
        head_yaw_ref = random.randrange(-15.0,15.0)
        head_pitch_ref = random.randrange(-7.0,7.0)
        RightShoulderPitch_ref = random.randrange(-100,-80)
        RightShoulderRoll_ref = random.randrange(-75,-65)
        LeftShoulderPitch_ref = random.randrange(80,100)
        LeftShoulderRoll_ref = random.randrange(-75,-65)
        
        #wait for ros service        
        rospy.wait_for_service('/qt_robot/motors/setVelocity')
                   
        try:
            
            setVelocity('HeadPitch',0)
            
            #We define a new Float64MultiArray message for the head motors
            href = Float64MultiArray()
            
            #We add data that we want to publish to it            
            href.data = [head_yaw_ref, head_pitch_ref]

            #We publish the data and order the head motors            
            head_pub.publish(href)

            #We add a random sleep time so that the body's motion becomes more smooth            
            rospy.sleep(random.choice([0,np.random.uniform(1,1.5)]))

            #We do the same thing for the left and right hand motors
            r_ref = Float64MultiArray()
            l_ref = Float64MultiArray()
            r_ref.data = [RightShoulderPitch_ref, RightShoulderRoll_ref,-20]
            l_ref.data = [LeftShoulderPitch_ref, LeftShoulderRoll_ref,-20]
            right_pub.publish(r_ref)
            left_pub.publish(l_ref)
            
            #We reduce the velocity of the motors' movement
            setVelocity(['left_arm', 'right_arm', 'HeadPitch'],2)

        #Continue if the user write something on the keyboard          
        except KeyboardInterrupt:
            
            pass
    rospy.loginfo("finished!")

    #Show to the user when the script is no more running   
    rospy.loginfo("No more random moves!") 

