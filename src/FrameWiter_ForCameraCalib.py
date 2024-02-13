import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge


bridge=CvBridge()

if __name__ == '__main__':
    frame_number=1
    #rate=rospy.Rate(140)  
    def frameCallback(data):
        global frame_number
        file_name="chessboard_images_right/frame"+str(frame_number)+".jpg"
        print("Test")
        curr_frame=bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        cv2.imshow('CurrFrame',curr_frame)
        cv2.imwrite(file_name,curr_frame)
        cv2.waitKey(1)
        frame_number+=1

    rospy.Subscriber('ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed', CompressedImage, frameCallback)
    rospy.init_node('ExpertPlayback')
    rospy.spin()