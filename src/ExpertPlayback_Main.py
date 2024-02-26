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


RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'




if __name__ == '__main__':
    

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer=Renderer.Renderer()

    #Initializing rospy subscriber

    rospy.init_node('ExperPlayback')
    rospy.Rate(10000)

    rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackRight,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackLeft,queue_size=1,buff_size=2**18)

    #Running Application
    tool_renderer.run(delay=100)