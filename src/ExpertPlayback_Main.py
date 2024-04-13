#import rospy
#from sensor_msgs.msg import CompressedImage
import cv2
#from cv_bridge import CvBridge
import os
from include import Renderer
import multiprocessing

#Ros and frame grabber imports
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs.msg import Joy


RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
Clutch_Topic='footpedals/clutch'
Camera_Topic='footpedals/camera'




if __name__ == '__main__':
    
    #Initializing rospy

    rospy.init_node('ExpertPlayback')
    rospy.Rate(10000)

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer=Renderer.Renderer()

    rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackRight,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackLeft,queue_size=1,buff_size=2**18)

    #rospy.Subscriber(Clutch_Topic,Joy,tool_renderer.clutchPedalCallback,queue_size=1,buff_size=1000000)
    #rospy.Subscriber(Camera_Topic,Joy,tool_renderer.cameraPedalCallback,queue_size=1,buff_size=1000000)
    #Running Application
    tool_renderer.run(delay=100)