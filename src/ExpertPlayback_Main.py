import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

bridge=CvBridge()

if __name__ == '__main__':
    
    #rate=rospy.Rate(140)

    def callback(data):
        print("Test")
        cv_image=bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        #cv_image=bridge.imgmsg_to_cv2(data,'passthrough')
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)
    

    rospy.Subscriber('ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed', CompressedImage, callback)
    rospy.init_node('ExpertPlayback')
    rospy.spin()