
#Imports

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
# import cv2
import numpy as np
import dvrk 
import sys
from scipy.spatial.transform import Rotation as R
import os
from utils import arm, mtm, footpedals
import time
import json



if __name__ == '__main__':
    rospy.init_node('playback_publisher')
    rate=rospy.Rate(140)

    PSM1=arm.robot('PSM1')
    PSM3=arm.robot('PSM3')

    def callback_dummy(data):
        PSM1_jp=PSM1.get_current_jaw_position()
        print(PSM1_jp)
    

    rospy.Subscriber('PSM1/measured_cp', PoseStamped, callback_dummy, queue_size = 1, buff_size = 1000000)

    rospy.spin()
