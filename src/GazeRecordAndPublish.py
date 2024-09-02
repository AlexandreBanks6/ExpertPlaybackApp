import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32
import os
import time

GAZE_ROOT='../resources/Motions/'


#When you start running, it starts recording and publishing


MAX_FRAME_FAIL_COUNT=50 #If we fail to grab 50 frames consecutively, stop running
#Sets the frame size of the gaze tracker, we get the combined frame through one channel
GAZE_FRAME_WIDTH=960
GAZE_FRAME_HEIGHT=540
MIDDLE_FRAME=int(GAZE_FRAME_WIDTH/2)
FRAME_FPS=30
#ROS Topic to Publish GAZE #
Gaze_Number_Topic='ExpertPlayback/GazeFrameNumber'
filecount_Topic='ExpertPlayback/fileCount' #What file number we should be saving to



class GazeRecorder:
    def __init__(self):
        #Booleans and Vars
        self.frame_count=0
        self.frame_fail_count=0
        self.file_count=1 #File number that we save gaze to
        self.gaze_filename=None
        print("Booleans")
        #Sets up ROS Node and Publisher
        rospy.init_node('GazeTracker')
        rospy.Rate(1000)
        #Publishes the frame count:
        try:
            self.gaze_num_publisher=rospy.Publisher(name = Gaze_Number_Topic,data_class=Int32,queue_size=10)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            exit()
        
        try:
            self.gaze_num_publisher.publish(self.frame_count)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            exit()
        
        #Subscriber to read the file count # to write files to
        try:
            rospy.Subscriber(name = filecount_Topic, data_class=Int32, callback=self.filecountCallback,queue_size=10,buff_size=2**8)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            exit()


        ##Get User Input for Video Channel (default 0)
        device_number=input("Input Gaze Video Device Number (default 0): ")

        #Sets up Video Capture
        self.cap=cv2.VideoCapture(int(device_number))
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB,0) #Does not automatically convert to RGB, reads as grayscale
        self.cap.set(cv2.CAP_PROP_FPS,FRAME_FPS)


        #Checks if the device is available
        if not self.cap.isOpened:
            print("Error: Could not open video device " +str(device_number)+", try again")
            exit()


        ####Sets Window to View Gaze####
        cv2.namedWindow("Gaze Video: ", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Gaze Video: ",GAZE_FRAME_WIDTH,GAZE_FRAME_HEIGHT)
        #Starts the gaze recording:
        self.initGazeRecording()
        self.new_time=time.time()
        self.old_time=time.time()



    def initGazeRecording(self):

        if self.gaze_filename is None:
            if not os.path.exists(GAZE_ROOT+'PC2'):
                os.makedirs(GAZE_ROOT+'PC2')
            #Set filename for gaze video
            self.gaze_filename=GAZE_ROOT+'PC2/Gaze_Video_'+str(self.file_count)+'.avi'
            fourcc=cv2.VideoWriter_fourcc(*'MJPG')
            self.gaze_video_writer=cv2.VideoWriter(self.gaze_filename,fourcc,FRAME_FPS,(GAZE_FRAME_WIDTH,GAZE_FRAME_HEIGHT))

    def filecountCallback(self,data):
        if self.file_count != data.data: #New file being written and we update video writer and filename
            if self.gaze_filename is not None: #If we are updating to a new file, we release the old video writer object
                self.gaze_video_writer.release()
            self.file_count=data.data
            self.gaze_filename=GAZE_ROOT+'PC2/Gaze_Video_'+str(self.file_count)+'.avi'
            fourcc=cv2.VideoWriter_fourcc(*'MJPG')
            self.gaze_video_writer=cv2.VideoWriter(self.gaze_filename,fourcc,FRAME_FPS,(GAZE_FRAME_WIDTH,GAZE_FRAME_HEIGHT))

if __name__ == '__main__':
    print("Started Gaze Recording App")
    try:
        gaze_record_obj=GazeRecorder()
    except Exception as e:
        print("Error: "+str(e))
        exit()

    while not rospy.is_shutdown():
        ret,frame=gaze_record_obj.cap.read()

        if not ret: #Failed to grab frame
            frame_fail_count+=1
            if frame_fail_count>=MAX_FRAME_FAIL_COUNT:
                print("Gaze Tracker Not Grabbing Frames")
                break
        else:
            frame_fail_count=0 #Resets the failed frame grabs counter

            gray_chan,_=cv2.split(frame)  #Extracts the grayscale frame
            gray_chan=cv2.resize(gray_chan,(GAZE_FRAME_WIDTH, GAZE_FRAME_HEIGHT),cv2.INTER_LINEAR) #Resizes to proper dimensions

            #Flips left/right sides so it matches with left/right eyes
            left_half=gray_chan[:,:MIDDLE_FRAME]
            right_half=gray_chan[:,MIDDLE_FRAME:]
            gray_chan=np.concatenate((right_half,left_half),axis=1)
            bgr_frame=cv2.cvtColor(gray_chan,cv2.COLOR_GRAY2BGR) #Need to write a BGR frame to the video
            

            #Increment frame counter and Publish the ROS Topic
            gaze_record_obj.frame_count+=1 
            try:
                gaze_record_obj.gaze_num_publisher.publish(gaze_record_obj.frame_count)
            except rospy.ROSInterruptException:
                print("ROS Node Interrupted")
                break
            except Exception as e:
                print("Unexpected Error: "+str(e))
                break
            gaze_record_obj.new_time=time.time()
            print("deltaTime="+str(gaze_record_obj.new_time-gaze_record_obj.old_time))
            gaze_record_obj.gaze_video_writer.write(bgr_frame)
            gaze_record_obj.old_time=gaze_record_obj.new_time
            
            cv2.imshow("Gaze Video: ",gray_chan) #Showing image
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    gaze_record_obj.gaze_video_writer.release()
    gaze_record_obj.cap.release()
    cv2.destroyAllWindows()
