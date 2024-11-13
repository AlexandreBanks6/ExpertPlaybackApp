'''
This script runs on a second computer to be used to offload computation from maing computer running "ExpertPlayback_Main.py"
It has two functions:
1) Computing and publishing the left camera to scene (lc_T_s) and right camera to scene (rc_T_s) transforms
2) Recording the pc2 time, the ecm_frame_number, the gaze frame number as well as saving the ecm frames and the gaze frames
Uses a tkinter interface
'''
import rospy
import numpy as np
from datetime import datetime
#import Tkinter as tk
import tkinter as tk
import cv2
import time


from include import DataLogger
from include import ArucoTracker_ForPC2
#from ROS_Msgs.msg import Mat4

#ROS Topic Conversions
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String

print("Imports Done")


##################ROS Topics####################
#Image Topic Subscriptions
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'

#PC 2 Time Subscription
PC2_Time_Topic='ExpertPlayback/pc2Time' #Published from PC2 (subscribed to by PC1)
filecount_Topic='ExpertPlayback/fileCount' #Published from PC1 (subscribed to by PC2)
lc_T_s_Topic='ExpertPlayback/lc_T_s' #Published from PC2 (subscribed to by PC1)
rc_T_s_Topic='ExpertPlayback/rc_T_s' #Published from PC2 (subscribed to by PC1)
Gaze_Number_Topic='ExpertPlayback/GazeFrameNumber'

MOTIONS_ROOT='../resources/Motions/'
ECM_FRAME_WIDTH_DESIRED=1024
ECM_FRAME_HEIGHT_DESIRED=722

class PC2RecordAndPublish:
    def __init__(self):
        
        #AruCo Tracking Objects
        self.aruco_tracker_left=ArucoTracker_ForPC2.ArucoTracker(self,'left')
        self.aruco_tracker_right=ArucoTracker_ForPC2.ArucoTracker(self,'right')
        

        #Datalogger Objects
        self.pc2_datalogger=DataLogger.DataLogger(self)

        #CV Bridge
        self.bridge=CvBridge()

        #Array Message
        self.array_msg=Float32MultiArray()

        #PC2 Time Message
        self.PC2_time_message=String()


        #########Set Up TKinter Window#########
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App PC2")

        #Configure Grid
        #self.gui_window.rowconfigure([0,1],weight=1)
        #self.gui_window.columnconfigure([0,1],weight=1)
        self.gui_window.minsize(100,50)

        #Title at top
        self.welcome_text=tk.Label(self.gui_window,text="Welcome to the dVRK Playback App PC2 Recorder/Publisher")
        self.welcome_text.grid(row=0,column=0,sticky='n')

        #Publish Topics Button
        self.publish_camtransformButton=tk.Button(self.gui_window,text="Publish Camera-to-Scene Transform (lc_T_s & rc_T_s)",command=self.publishCamToSceneTransformCallback)
        self.publish_camtransformButton.grid(row=1,column=0,sticky="nsew")

        #Record PC2 Data
        self.recordPC2DataButton=tk.Button(self.gui_window,text="Record PC2 Data (ecm frames, gaze frames, and lc_T_s/rc_T_s)",command=self.recordPC2DataCallback)
        self.recordPC2DataButton.grid(row=1,column=1,sticky="nsew")

        #Show Aruco/Tracking Button
        self.showArucoTrackingButton=tk.Button(self.gui_window,text="Show AruCo Tracking/Pose",command=self.showPoseTrackingCallback)
        self.showArucoTrackingButton.grid(row=2,column=0,sticky="nsew")
        
        ###########Booleans and Vars############
        self.is_publish=False
        self.ranOnce=0
        self.is_Record=False
        self.file_count=1
        self.is_showPoseTracking=False

        self.new_time=time.time()
        self.old_time=time.time()

        self.left_right='left'

        #Used to check if any frames are coming in
        self.right_frame_number_check=0 
        self.left_frame_number_check=0

        #Used to count number of frames received since recording started to align csv rows with video frames
        self.left_ecm_frame_number=0 #1 is the first frame
        self.right_ecm_frame_number=0       

        self.gaze_frame_number=0


        ##########Transform Initialize###########
        self.lc_T_s=np.empty((4,4),dtype='float32')
        self.lc_T_s[:]=np.nan
        self.rc_T_s=np.empty((4,4),dtype='float32')
        self.rc_T_s[:]=np.nan


        ###############ECM Frames################
        self.frame_right=None
        self.frame_left=None



        ################ROS Init################
        rospy.init_node('ExpertPlayback_PC2')
        rospy.Rate(1000)

        #Right/Left Frames Subscribers
        try:
            rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackRight,queue_size=1,buff_size=2**18)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()

        try:
            rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackLeft,queue_size=1,buff_size=2**18)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()

        #Cam-to-Scene Transform Publisher
        try:
            self.lc_T_s_publisher=rospy.Publisher(name = lc_T_s_Topic,data_class=Float32MultiArray,queue_size=10)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()

        try:
            self.rc_T_s_publisher=rospy.Publisher(name = rc_T_s_Topic,data_class=Float32MultiArray,queue_size=10)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()

        #PC2 Time Publisher
        try:
            self.PC2_time_publiser=rospy.Publisher(name = PC2_Time_Topic,data_class=String,queue_size=10)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()

        #file count (for saving data and syncing files) subscriber
        try:
            rospy.Subscriber(name = filecount_Topic, data_class=Int32, callback=self.filecountCallback,queue_size=1,buff_size=2**8)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()

        #Gaze Number Topic
        try:
            rospy.Subscriber(name = Gaze_Number_Topic, data_class=Int32, callback=self.gazeCountCallback,queue_size=1,buff_size=2**6)
        except rospy.ROSInterruptException:
            print("ROS Node Interrupted")
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        except Exception as e:
            print("Unexpected Error: "+str(e))
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            exit()
        ###############Main Loop################
        self.gui_window.after(1,self.mainCallback)
        self.gui_window.mainloop()
        

    def frameCallbackRight(self,data):
        self.right_frame_number_check+=1
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.frame_right=cv2.resize(self.frame_right,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
                   
        if self.is_Record:
            self.pc2_datalogger.right_video_writer.write(self.frame_right)
            self.right_ecm_frame_number+=1
            print("Right Frame Record")
        

    def frameCallbackLeft(self,data):
        self.left_frame_number_check+=1
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.frame_left=cv2.resize(self.frame_left,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
 
        if self.is_Record:
            #init_time=time.time()
            self.pc2_datalogger.left_video_writer.write(self.frame_left)
            #after_time=time.time()
            #print("Time Diff="+str(after_time-init_time))

            self.left_ecm_frame_number+=1
            print("Left Frame Record")
        

    def publishCamToSceneTransformCallback(self):
        self.is_publish=not self.is_publish

    
    def filecountCallback(self,data):
        self.file_count=data.data

    def recordPC2DataCallback(self):
        self.is_Record=not self.is_Record
        
        if self.is_Record: #We are starting recording
            #Init the csv
            self.pc2_datalogger.initRecording_PC2(MOTIONS_ROOT,self.file_count)
            #init frame numbers
            self.left_ecm_frame_number=0
            self.right_ecm_frame_number=0

    def showPoseTrackingCallback(self):
        self.is_showPoseTracking=not self.is_showPoseTracking
        if not self.is_showPoseTracking:
            cv2.destroyAllWindows()

    def gazeCountCallback(self,data):
        self.gaze_frame_number=data.data


    def mainCallback(self):
        if not rospy.is_shutdown():
            if self.left_frame_number_check>0 and self.right_frame_number_check>0:
                if self.is_publish: #Publish the lc_T_s and rc_T_s transforms
                    self.frame_left=cv2.resize(self.frame_left,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
                    self.frame_right=cv2.resize(self.frame_right,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
                    lc_T_s=self.aruco_tracker_left.calibrateSceneDirectNumpy(self.frame_left.copy(),self.is_showPoseTracking,'left pose')
                    rc_T_s=self.aruco_tracker_right.calibrateSceneDirectNumpy(self.frame_right.copy(),self.is_showPoseTracking,'right pose')


                    if lc_T_s is not None: #Updates values is there is an AruCo in view
                        self.lc_T_s=lc_T_s #Numpy array of transform
                        #print("lc_T_s: "+str(lc_T_s))

                    
                    if rc_T_s is not None:
                        self.rc_T_s=rc_T_s #Numpy array of transform
                        #print("rc_T_s: "+str(rc_T_s))


                    #Publishing the lc_T_s and rc_T_s messages
                    self.array_msg.data=self.lc_T_s.flatten().tolist()
                    try:
                        self.lc_T_s_publisher.publish(self.array_msg)
                    except rospy.ROSInterruptException:
                        print("ROS Node Interrupted")
                        self.pc2_datalogger.left_video_writer.release()
                        self.pc2_datalogger.right_video_writer.release()
                        cv2.destroyAllWindows()
                        exit()
                    except Exception as e:
                        print("Unexpected Error: "+str(e))
                        self.pc2_datalogger.left_video_writer.release()
                        self.pc2_datalogger.right_video_writer.release()
                        cv2.destroyAllWindows()
                        exit()
                    


                    self.array_msg.data=self.rc_T_s.flatten().tolist()
                    try:
                        self.rc_T_s_publisher.publish(self.array_msg)
                    except rospy.ROSInterruptException:
                        print("ROS Node Interrupted")
                        self.pc2_datalogger.left_video_writer.release()
                        self.pc2_datalogger.right_video_writer.release()
                        cv2.destroyAllWindows()
                        exit()
                    except Exception as e:
                        print("Unexpected Error: "+str(e))
                        self.pc2_datalogger.left_video_writer.release()
                        self.pc2_datalogger.right_video_writer.release()
                        cv2.destroyAllWindows()
                        exit()
                    #Gets the pc2 time
                    pc2_time=datetime.now().time()

                    #Publishes the pc2 time
                    self.PC2_time_message.data=str(pc2_time)
                    try:
                        self.PC2_time_publiser.publish(self.PC2_time_message)
                    except rospy.ROSInterruptException:
                        print("ROS Node Interrupted")
                        self.pc2_datalogger.left_video_writer.release()
                        self.pc2_datalogger.right_video_writer.release()
                        cv2.destroyAllWindows()
                        exit()
                    except Exception as e:
                        print("Unexpected Error: "+str(e))
                        self.pc2_datalogger.left_video_writer.release()
                        self.pc2_datalogger.right_video_writer.release()
                        cv2.destroyAllWindows()
                        exit()


                if self.is_Record: #Record data
                    if not self.is_publish: #Publish is not on, we must compute the cam-to-scene transform
                        self.frame_left=cv2.resize(self.frame_left,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
                        self.frame_right=cv2.resize(self.frame_right,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)

                        lc_T_s=self.aruco_tracker_left.calibrateSceneDirectNumpy(self.frame_left.copy(),self.is_showPoseTracking,'left pose')
                        #init_time=time.time()
                        rc_T_s=self.aruco_tracker_right.calibrateSceneDirectNumpy(self.frame_right.copy(),self.is_showPoseTracking,'right pose')
                        #after_time=time.time()
                        #print("Time Diff="+str(after_time-init_time))
                        pc2_time=datetime.now().time()
                    

                    # self.new_time=time.time()
                    # print("Time Diff Overall= "+str(self.new_time-self.old_time))
                    # self.old_time=self.new_time

                    #Writes the Row to the Data CSV  
                    self.pc2_datalogger.writeRow_PC2(pc2_time,self.left_ecm_frame_number,self.right_ecm_frame_number,self.gaze_frame_number,lc_T_s,rc_T_s)
                    
            self.gui_window.after(1,self.mainCallback)
        else:
            #ROS is shutdown
            self.pc2_datalogger.left_video_writer.release()
            self.pc2_datalogger.right_video_writer.release()
            cv2.destroyAllWindows()
            self.gui_window.quit()
            exit()




if __name__=='__main__':
    print("Started PC2 Record and Publish")
    try:
        pc2_recorder_publisher=PC2RecordAndPublish()
    except Exception as e:
        print("Error: "+str(e))
        exit()
        

    if pc2_recorder_publisher.pc2_datalogger is not None:
        pc2_recorder_publisher.pc2_datalogger.left_video_writer.release()
        pc2_recorder_publisher.pc2_datalogger.right_video_writer.release()
    cv2.destroyAllWindows()

