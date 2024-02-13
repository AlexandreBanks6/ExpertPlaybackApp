import rospy
from sensor_msgs.msg import CompressedImage
from include import ECM 

ECM=ECM.ECM_Class()

if __name__ == '__main__':
    
    #rate=rospy.Rate(140)  

    #########Initialize ROS events############

    #Left ECM
    rospy.Subscriber('ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed', CompressedImage, ECM.frameCallback)
    rospy.init_node('ExpertPlayback')


    #Instrument Renderings
    print("Entering")
    #ECM.renderScene()

    rospy.spin()