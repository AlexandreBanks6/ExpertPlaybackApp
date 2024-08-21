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
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray 






if __name__ == '__main__':
    
    #Initializing rospy

    rospy.init_node('ExpertPlayback')
    rospy.Rate(10000)

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer=Renderer()

    rospy.Subscriber(name = Renderer.RightFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackRight,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = Renderer.LeftFrame_Topic, data_class=CompressedImage, callback=tool_renderer.frameCallbackLeft,queue_size=1,buff_size=2**18)
    rospy.Subscriber(name = Renderer.PC2_Time_Topic,data_class=String,callback=tool_renderer.pc2TimeCallback,queue_size=1,buff_size=2**7)
    rospy.Subscriber(name = Renderer.lc_T_s_Topic,data_class=Float32MultiArray, callback=tool_renderer.lcTs_Callback,queue_size=1,buff_size=2**10)
    rospy.Subscriber(name = Renderer.rc_T_s_Topic,data_class=Float32MultiArray, callback=tool_renderer.rcTs_Callback,queue_size=1,buff_size=2**10)

    tool_renderer.run(frame_rate=1000)