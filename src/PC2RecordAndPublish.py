'''
This script runs on a second computer to be used to offload computation from maing computer running "ExpertPlayback_Main.py"
It has two functions:
1) Computing and publishing the left camera to scene (lc_T_s) and right camera to scene (rc_T_s) transforms
2) Recording the pc2 time, the ecm_frame_number, the gaze frame number as well as saving the ecm frames and the gaze frames
Uses a tkinter interface
'''
import rospy
import glm
import numpy as np
from datetime import datetime
import csv
import os
import tkinter as tk
import dvrk
import PyKDL
import cv2


from include import DataLogger
from include import utils
from include import ArucoTracker
from include import Renderer
#from ROS_Msgs.msg import Mat4

#ROS Topic Conversions
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String

class PC2RecordAndPublish:
    def __init__(self):

        #AruCo Tracking Objects
        self.aruco_tracker_left=ArucoTracker.ArucoTracker(self,'left')
        self.aruco_tracker_right=ArucoTracker.ArucoTracker(self,'right')

        #Datalogger Objects
        self.pc2_datalogger=DataLogger.DataLogger(self)

        #CV Bridge
        self.bridge=CvBridge()

        #Array Message
        self.array_msg=Float32MultiArray()


        #########Set Up TKinter Window#########
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App PC2")

        #Configure Grid
        self.gui_window.rowconfigure([0,1],weight=1)
        self.gui_window.columnconfigure([0,1],weight=1)
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

        
        ###########Booleans and Vars############
        self.is_publish=False
        self.ranOnce=0
        self.is_Record=False
        self.file_count=1

        self.left_ecm_frame_number=0 #1 is the first frame
        self.right_ecm_frame_number=0
        self.left_gaze_frame_number=0
        self.right_gaze_frame_number=0


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
        rospy.Rate(100)

        #Right/Left Frames Subscribers
        rospy.Subscriber(name = Renderer.RightFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackRight,queue_size=1,buff_size=2**18)
        rospy.Subscriber(name = Renderer.LeftFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackLeft,queue_size=1,buff_size=2**18)

        #Cam-to-Scene Transform Publisher
        self.lc_T_s_publisher=rospy.Publisher(name = Renderer.lc_T_s_Topic,data_class=Float32MultiArray,queue_size=10)
        self.rc_T_s_publisher=rospy.Publisher(name = Renderer.rc_T_s_Topic,data_class=Float32MultiArray,queue_size=10)

        #file count (for saving data and syncing files) subscriber
        rospy.Subscriber(name = Renderer.filecount_Topic, data_class=Int32, callback=self.filecountCallback,queue_size=1,buff_size=2**6)

        ###############Main Loop################
        self.gui_window.after(1,self.mainCallback)
        self.gui_window.mainloop()

    def frameCallbackRight(self,data):
        self.right_ecm_frame_number+=1
        self.frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.pc2_datalogger.right_video_writer.write(self.frame_right)
        

    def frameCallbackLeft(self,data):
        self.left_ecm_frame_number+=1
        self.frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.pc2_datalogger.left_video_writer.write(self.frame_left)
        

    def publishCamToSceneTransformCallback(self):
        self.is_publish=not self.is_publish

    
    def filecountCallback(self,data):
        self.file_count=data.data

    def recordPC2DataCallback(self):
        self.is_Record=not self.is_Record

        #Init the csv
        self.pc2_datalogger.initRecording_PC2(Renderer.MOTIONS_ROOT,self.file_count)



    def mainCallback(self):
        if self.left_ecm_frame_number>0 and self.right_ecm_frame_number>0:
            if self.is_publish: #Publish the lc_T_s and rc_T_s transforms
                lc_T_s=self.aruco_tracker_left.calibrateSceneDirectNumpy(self.frame_left)
                rc_T_s=self.aruco_tracker_right.calibrateSceneDirectNumpy(self.frame_right)
                if lc_T_s is not None: #Updates values is there is an AruCo in view
                    self.lc_T_s=lc_T_s #Numpy array of transform
                
                if rc_T_s is not None:
                    self.rc_T_s=rc_T_s #Numpy array of transform

                #Publishing the lc_T_s and rc_T_s messages
                self.array_msg.data=self.lc_T_s.flatten().tolist()
                self.lc_T_s_publisher.publish(self.array_msg)

                self.array_msg.data=self.rc_T_s.flatten().tolist()
                self.rc_T_s_publisher.publish(self.array_msg)

            if self.is_Record: #Record data
                if not self.is_publish: #Publish is not on, we must compute the cam-to-scene transform
                    lc_T_s=self.aruco_tracker_left.calibrateSceneDirectNumpy(self.frame_left)
                    rc_T_s=self.aruco_tracker_right.calibrateSceneDirectNumpy(self.frame_right)
                    if lc_T_s is not None: #Updates values is there is an AruCo in view
                        self.lc_T_s=lc_T_s #Numpy array of transform
                    
                    if rc_T_s is not None:
                        self.rc_T_s=rc_T_s #Numpy array of transform
                
                #Gets the pc2 time
                pc2_time=datetime.now().time()
                
                #Increments the ecm frame number:                
                self.pc2_datalogger.writeRow_PC2(pc2_time,self.left_ecm_frame_number,self.right_ecm_frame_number,self.left_gaze_frame_number,self.right_gaze_frame_number,self.lc_T_s,self.rc_T_s)
            

            

if __name__=='main':
    pc2_recorder_publisher=PC2RecordAndPublish()
    pc2_recorder_publisher.pc2_datalogger.left_video_writer.release()
    pc2_recorder_publisher.pc2_datalogger.right_video_writer.release()
    cv2.destroyAllWindows()

    