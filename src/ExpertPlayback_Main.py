import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import os
from include import Renderer
import multiprocessing

#We implement gamma correction and mipmap correction

#ECM=ECM.ECM_Class()

class MainClass():
    def __init__(self):
        self.tool_renderer_right=None
        self.tool_renderer_left=None
        p1=multiprocessing.Process(target=self.initRightRenderer)
        p2=multiprocessing.Process(target=self.initLeftRenderer)
        p1.start()
        p2.start()
        p1.join()
        p2.join()
    def initRightRenderer(self):
        self.tool_renderer_right=Renderer.Renderer()
    def initLeftRenderer(self):
        self.tool_renderer_left=Renderer.Renderer()


if __name__ == '__main__':
    

    #Initialize the Renderer Class for both right and left eyes
    tool_renderer_right=Renderer.Renderer()
    tool_renderer_left=Renderer.Renderer()
    #Windows=MainClass()

    #Main Render loop
    while True:
        p1=multiprocessing.Process(target=tool_renderer_right.updateWindow(delay=60))
        p2=multiprocessing.Process(target=tool_renderer_left.updateWindow(delay=60))
        p1.start()
        p2.start()
        p1.join()
        p2.join() 
        #tool_renderer_left.updateWindow(delay=60)

    #rate=rospy.Rate(140)  

    #########Initialize ROS events############

    #Left ECM
    #rospy.Subscriber('ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed', CompressedImage, ECM.frameCallback)
    #rospy.init_node('ExpertPlayback')


    #Instrument Renderings
    #print("Entering")
    #ECM.renderScene()

    #rospy.spin()