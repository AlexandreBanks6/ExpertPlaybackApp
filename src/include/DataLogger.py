<<<<<<< HEAD
#Class to record robot kinematics/video from PC1 for expert playback app
#Always start recording on PC1 before PC2

import glm
import numpy as np
import cv2
from datetime import datetime
import csv
import os
import serial
import time
import sys

CONSOLE_VIEWPORT_WIDTH=1024
CONSOLE_VIEWPORT_HEIGHT=722


#File Column Titles:
repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"] #First number is row of R, second number is column
MOTION_HEADER_PC1=["Task Time","PC1 Time","PC2 Time","Gaze Calib","s_T_psm1"]+repeat_string+["s_T_psm3"]+repeat_string+\
    ["cart_T_ecm"]+repeat_string+["ecm_T_psm1"]+repeat_string+["ecm_T_psm3"]+repeat_string+\
            ["psm1_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+["psm3_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+\
            ["ecm_joints"]+["q1","q2","q3","q4"]+["ecmi_T_ecm"]+repeat_string+\
                ["Sample #","Task Duration (s)","Collisions (binary)","Collisions (increments)","Cumulative Collision Time (s)"]

MOTION_HEADER_PC2=["PC2 Time","Left ECM Frame #","Right ECM Frame #","Gaze Frame #","lc_T_s"]+repeat_string+["rc_T_s"]+repeat_string

FRAME_FPS=27


##Syncrhonizing this with Arduino for collision detection
##Arduino states are:
##Enter toggles state: If on, sends data, if off sends "Task Stopped"

class DataLogger:
    def __init__(self,app):
        print('Init Datalogger')
        self.record_filename_pc1=None
        self.record_filename_pc2=None
        self.left_ecm_filename_pc2=None
        self.right_ecm_filename_pc2=None
        self.left_video_writer=None
        self.right_video_writer=None
        self.file_count=1
        self.app=app

        #Initializng Arduino Reader Things
        #self.ser_port=serial.Serial('/dev/ttyACM0',57600) #Use 'COM5' or similar if windows
        self.arduino_list=["NaN"]*5 #Inits this value
        
    def initRecording_PC1(self,root_path):
        #Check if path exists
        if not os.path.exists(root_path+'PC1'):
            os.makedirs(root_path+'PC1')

        #Initialize a new CSV file to save PC1 data to
        file_name=root_path+'PC1/Data_PC1_'+str(self.file_count)+'.csv'
        #Gets a new filename
        while True:
            if os.path.isfile(file_name):
                self.file_count+=1
                file_name=root_path+'PC1/Data_PC1_'+str(self.file_count)+'.csv'                
            else:
                break
        
        self.record_filename_pc1=file_name  #Creates a new motion csv



        #Store all transforms that do not change
        with open(self.record_filename_pc1,'w',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(MOTION_HEADER_PC1)
            writer_object.writerow([""]) #Blank Space

            #Writing si_T_lci (scene to left camera initial)
            lci_T_si_numpy=np.array(glm.transpose(self.app.lci_T_si).to_list(),dtype='float32')
            lci_T_si_list=self.convertHomogeneousToCSVROW(lci_T_si_numpy)
            writer_object.writerow(["lci_T_si","","","",""]+lci_T_si_list)

            #Writing si_T_rci (scene to right camera initial)
            rci_T_si_numpy=np.array(glm.transpose(self.app.rci_T_si).to_list(),dtype='float32')
            rci_T_si_list=self.convertHomogeneousToCSVROW(rci_T_si_numpy)
            writer_object.writerow(["rci_T_si","","","",""]+rci_T_si_list)

            #Writing lc_T_ecm (hand-eye left)
            ecm_T_lc_numpy=np.array(glm.transpose(self.app.ecm_T_lc).to_list(),dtype='float32')
            ecm_T_lc_list=self.convertHomogeneousToCSVROW(ecm_T_lc_numpy)
            writer_object.writerow(["ecm_T_lc","","","",""]+ecm_T_lc_list)

            #Writing rc_T_ecm (hand-eye right)
            ecm_T_rc_numpy=np.array(glm.transpose(self.app.ecm_T_rc).to_list(),dtype='float32')
            ecm_T_rc_list=self.convertHomogeneousToCSVROW(ecm_T_rc_numpy)
            writer_object.writerow(["ecm_T_rc","","","",""]+ecm_T_rc_list)

            #Writing cart_T_ecmi (robot base to ecm initial)
            cart_T_ecmi_numpy=np.array(glm.transpose(self.app.cart_T_ecmi).to_list(),dtype='float32')
            cart_T_ecmi_list=self.convertHomogeneousToCSVROW(cart_T_ecmi_numpy)
            writer_object.writerow(["cart_T_ecmi","","","",""]+cart_T_ecmi_list)

            #ecmac_T_ecmrep_psm1 & ecmac_T_ecmrep_psm3 (API error correction)
            ecmac_T_ecmrep_psm1_numpy=np.array(glm.transpose(self.app.ecmac_T_ecmrep_psm1).to_list(),dtype='float32')
            ecmac_T_ecmrep_psm1_list=self.convertHomogeneousToCSVROW(ecmac_T_ecmrep_psm1_numpy)
            writer_object.writerow(["ecmac_T_ecmrep_psm1","","","",""]+ecmac_T_ecmrep_psm1_list)

            ecmac_T_ecmrep_psm3_numpy=np.array(glm.transpose(self.app.ecmac_T_ecmrep_psm3).to_list(),dtype='float32')
            ecmac_T_ecmrep_psm3_list=self.convertHomogeneousToCSVROW(ecmac_T_ecmrep_psm3_numpy)
            writer_object.writerow(["ecmac_T_ecmrep_psm3","","","",""]+ecmac_T_ecmrep_psm3_list)

            writer_object.writerow([""]) #Blank Space

            file_object.close()

        
        #Starts Arduino Recording
        #self.ser_port.write(b'\n')  #Sends enter to start 


    def stopRecording_PC1(self):
        self.ser_port.write(b'\n') #Sends Enter to stop command

    
    def initRecording_PC2(self,root_path,file_count):

        #file_count comes from ROS topic published by PC1

        #Check if directory exists, if it doesn't create it
        if not os.path.exists(root_path+'PC2'):
            os.makedirs(root_path+'PC2')

        #Initialize a new CSV file to save PC2 data to
        file_name=root_path+'PC2/Data_PC2_'+str(file_count)+'.csv'
        self.record_filename_pc2=file_name  #Creates a new data csv
        with open(self.record_filename_pc2,'w') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(MOTION_HEADER_PC2)
            file_object.close()

        #Sets the filename (and path) for the left/right ECM video
        self.left_ecm_filename_pc2=root_path+'PC2/LeftECM_PC2_'+str(file_count)+'.avi'
        self.right_ecm_filename_pc2=root_path+'PC2/RightECM_PC2_'+str(file_count)+'.avi'
        #initializes the video writer objects
        fourcc=cv2.VideoWriter_fourcc(*'MJPG')
        self.left_video_writer=cv2.VideoWriter(self.left_ecm_filename_pc2,fourcc,FRAME_FPS,(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT))
        self.right_video_writer=cv2.VideoWriter(self.right_ecm_filename_pc2,fourcc,FRAME_FPS,(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT))



    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                    str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                    str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                    str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
        
        
        return string_list
    
    def writeRow_PC1(self,task_time,pc1_time,pc2_time,gaze_calib,s_T_psm1,s_T_psm3,cart_T_ecm,ecm_T_psm1,ecm_T_psm3,psm1_joints,psm3_joints,ecm_joints,ecmi_T_ecm):
        #PC2 time is published as a ROS topic from PC2

        #s_T_psm1 & s_T_psm3
        if s_T_psm1 is not None:
            s_T_psm1_numpy=np.array(glm.transpose(s_T_psm1).to_list(),dtype='float32')
            s_T_psm1_list=self.convertHomogeneousToCSVROW(s_T_psm1_numpy)
        else:
            s_T_psm1_list=["NaN"]*12

        if s_T_psm3 is not None:
            s_T_psm3_numpy=np.array(glm.transpose(s_T_psm3).to_list(),dtype='float32')
            s_T_psm3_list=self.convertHomogeneousToCSVROW(s_T_psm3_numpy)
        else:
            s_T_psm3_list=["NaN"]*12


        #cart_T_ecm
        if cart_T_ecm is not None:
            cart_T_ecm_numpy=np.array(glm.transpose(cart_T_ecm).to_list(),dtype='float32')
            cart_T_ecm_list=self.convertHomogeneousToCSVROW(cart_T_ecm_numpy)
        else:
            cart_T_ecm_list=["NaN"]*12


        #ecm_T_psm1 & ecm_T_psm3
        if ecm_T_psm1 is not None:
            ecm_T_psm1_numpy=np.array(glm.transpose(ecm_T_psm1).to_list(),dtype='float32')
            ecm_T_psm1_list=self.convertHomogeneousToCSVROW(ecm_T_psm1_numpy)
        else:
            ecm_T_psm1_list=["NaN"]*12

        if ecm_T_psm3 is not None:
            ecm_T_psm3_numpy=np.array(glm.transpose(ecm_T_psm3).to_list(),dtype='float32')
            ecm_T_psm3_list=self.convertHomogeneousToCSVROW(ecm_T_psm3_numpy)
        else:
            ecm_T_psm3_list=["NaN"]*12

        
        #psm1_joints and psm3_joints

        if psm1_joints is not None:
            joint_list_psm1=[str(num) for num in psm1_joints]
        else:
            joint_list_psm1=["NaN"]*7

        if psm3_joints is not None:
            joint_list_psm3=[str(num) for num in psm3_joints]
        else:
            joint_list_psm3=["NaN"]*7

        
        #ecm_joints
        if ecm_joints is not None:
            joint_list_ecm=[str(num) for num in ecm_joints]
        else:
            joint_list_ecm=["NaN"]*4

        if pc2_time is None:
            pc2_time="NaN"

        if ecmi_T_ecm is not None:
            ecmi_T_ecm_numpy=np.array(glm.transpose(ecmi_T_ecm).to_list(),dtype='float32')
            ecmi_T_ecm_list=self.convertHomogeneousToCSVROW(ecmi_T_ecm_numpy)
        else:
            ecmi_T_ecm_list=["NaN"]*12

        if(self.ser_port.in_waiting()>0):
            #New data received from arduino
            ser_bytes=self.ser_port.readline()
            decoded_bytes=ser_bytes.decode("utf-8").strip()
            arduino_list=decoded_bytes.split(",")
            arduino_list=[int(item) for item in arduino_list]
            arduino_list[1]=arduino_list[1]/1000
            arduino_list[4]=arduino_list[4]/1000
            arduino_list=[str(item) for item in arduino_list]
            self.arduino_list=arduino_list  #Updates the arduino list






        #pc2_time is already a string
        #Write the row
        row_to_write=[str(task_time),str(pc1_time),pc2_time,str(gaze_calib),""]+s_T_psm1_list+[""]+s_T_psm3_list+[""]+\
        cart_T_ecm_list+[""]+ecm_T_psm1_list+[""]+ecm_T_psm3_list+[""]+joint_list_psm1+[""]+joint_list_psm3+[""]+\
        joint_list_ecm+[""]+ecmi_T_ecm_list+self.arduino_list

        with open(self.record_filename_pc1,'a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(row_to_write)
            file_object.close()

    def writeRow_PC2(self,pc2_time,left_ecm_frame_number,right_ecm_frame_number,gaze_frame_number,lc_T_s_numpy,rc_T_s_numpy):

        #lc_T_s & rc_T_s
        if lc_T_s_numpy is not None:
            lc_T_s_list=self.convertHomogeneousToCSVROW(lc_T_s_numpy)
        else:
            lc_T_s_list=["NaN"]*12

        if rc_T_s_numpy is not None:
            rc_T_s_list=self.convertHomogeneousToCSVROW(rc_T_s_numpy)
        else:
            rc_T_s_list=["NaN"]*12

        row_to_write=[str(pc2_time),str(left_ecm_frame_number),str(right_ecm_frame_number),str(gaze_frame_number),""]+lc_T_s_list+[""]+rc_T_s_list
        row_to_write=list(row_to_write)

        with open(self.record_filename_pc2,'a') as file_obj:
            writer_object=csv.writer(file_obj)
            writer_object.writerow(row_to_write)
            file_obj.close()

    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                    str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                    str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                    str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
        
        
        return string_list






    

    





=======
#Class to record robot kinematics/video from PC1 for expert playback app
#Always start recording on PC1 before PC2

import glm
import numpy as np
import cv2
from datetime import datetime
import csv
import os
import serial
import time
import sys

CONSOLE_VIEWPORT_WIDTH=1024
CONSOLE_VIEWPORT_HEIGHT=722


#File Column Titles:
repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"] #First number is row of R, second number is column
MOTION_HEADER_PC1=["Task Time","PC1 Time","PC2 Time","Gaze Calib","s_T_psm1"]+repeat_string+["s_T_psm3"]+repeat_string+\
    ["cart_T_ecm"]+repeat_string+["ecm_T_psm1"]+repeat_string+["ecm_T_psm3"]+repeat_string+\
            ["psm1_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+["psm3_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+\
            ["ecm_joints"]+["q1","q2","q3","q4"]+["ecmi_T_ecm"]+repeat_string+\
                ["Sample #","Task Duration (s)","Collisions (binary)","Collisions (increments)","Cumulative Collision Time (s)"]+\
                ["Clutch Press","MTMR Angle","MTML Angle"]

MOTION_HEADER_PC2=["PC2 Time","Left ECM Frame #","Right ECM Frame #","Gaze Frame #","lc_T_s"]+repeat_string+["rc_T_s"]+repeat_string

FRAME_FPS=27


##Syncrhonizing this with Arduino for collision detection
##Arduino states are:
##Enter toggles state: If on, sends data, if off sends "Task Stopped"

class DataLogger:
    def __init__(self,app):
        print('Init Datalogger')
        self.record_filename_pc1=None
        self.record_filename_pc2=None
        self.left_ecm_filename_pc2=None
        self.right_ecm_filename_pc2=None
        self.left_video_writer=None
        self.right_video_writer=None
        self.file_count=1
        self.app=app

        #Initializng Arduino Reader Things
        try:
            self.ser_port=serial.Serial('/dev/ttyACM0',57600) #Use 'COM5' or similar if windows
            print("Connected to arduino")
            self.arduino_connected=True
        except:
            print("Failed to connect to arduino")
            self.arduino_connected=False
        self.arduino_list=["NaN"]*5 #Inits this value
        
    def initRecording_PC1(self,root_path):
        #Check if path exists
        if not os.path.exists(root_path+'PC1'):
            os.makedirs(root_path+'PC1')

        #Initialize a new CSV file to save PC1 data to
        file_name=root_path+'PC1/Data_PC1_'+str(self.file_count)+'.csv'
        #Gets a new filename
        while True:
            if os.path.isfile(file_name):
                self.file_count+=1
                file_name=root_path+'PC1/Data_PC1_'+str(self.file_count)+'.csv'                
            else:
                break
        
        self.record_filename_pc1=file_name  #Creates a new motion csv



        #Store all transforms that do not change
        with open(self.record_filename_pc1,'w',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(MOTION_HEADER_PC1)
            writer_object.writerow([""]) #Blank Space

            #Writing si_T_lci (scene to left camera initial)
            lci_T_si_numpy=np.array(glm.transpose(self.app.lci_T_si).to_list(),dtype='float32')
            lci_T_si_list=self.convertHomogeneousToCSVROW(lci_T_si_numpy)
            writer_object.writerow(["lci_T_si","","","",""]+lci_T_si_list)

            #Writing si_T_rci (scene to right camera initial)
            rci_T_si_numpy=np.array(glm.transpose(self.app.rci_T_si).to_list(),dtype='float32')
            rci_T_si_list=self.convertHomogeneousToCSVROW(rci_T_si_numpy)
            writer_object.writerow(["rci_T_si","","","",""]+rci_T_si_list)

            #Writing lc_T_ecm (hand-eye left)
            ecm_T_lc_numpy=np.array(glm.transpose(self.app.ecm_T_lc).to_list(),dtype='float32')
            ecm_T_lc_list=self.convertHomogeneousToCSVROW(ecm_T_lc_numpy)
            writer_object.writerow(["ecm_T_lc","","","",""]+ecm_T_lc_list)

            #Writing rc_T_ecm (hand-eye right)
            ecm_T_rc_numpy=np.array(glm.transpose(self.app.ecm_T_rc).to_list(),dtype='float32')
            ecm_T_rc_list=self.convertHomogeneousToCSVROW(ecm_T_rc_numpy)
            writer_object.writerow(["ecm_T_rc","","","",""]+ecm_T_rc_list)

            #Writing cart_T_ecmi (robot base to ecm initial)
            cart_T_ecmi_numpy=np.array(glm.transpose(self.app.cart_T_ecmi).to_list(),dtype='float32')
            cart_T_ecmi_list=self.convertHomogeneousToCSVROW(cart_T_ecmi_numpy)
            writer_object.writerow(["cart_T_ecmi","","","",""]+cart_T_ecmi_list)

            #ecmac_T_ecmrep_psm1 & ecmac_T_ecmrep_psm3 (API error correction)
            ecmac_T_ecmrep_psm1_numpy=np.array(glm.transpose(self.app.ecmac_T_ecmrep_psm1).to_list(),dtype='float32')
            ecmac_T_ecmrep_psm1_list=self.convertHomogeneousToCSVROW(ecmac_T_ecmrep_psm1_numpy)
            writer_object.writerow(["ecmac_T_ecmrep_psm1","","","",""]+ecmac_T_ecmrep_psm1_list)

            ecmac_T_ecmrep_psm3_numpy=np.array(glm.transpose(self.app.ecmac_T_ecmrep_psm3).to_list(),dtype='float32')
            ecmac_T_ecmrep_psm3_list=self.convertHomogeneousToCSVROW(ecmac_T_ecmrep_psm3_numpy)
            writer_object.writerow(["ecmac_T_ecmrep_psm3","","","",""]+ecmac_T_ecmrep_psm3_list)

            writer_object.writerow([""]) #Blank Space

            file_object.close()

        
        #Starts Arduino Recording
        if self.arduino_connected:
            self.ser_port.write(b'\n')  #Sends enter to start 


    def stopRecording_PC1(self):
        if self.arduino_connected:
            self.ser_port.write(b'\n') #Sends Enter to stop command

    
    def initRecording_PC2(self,root_path,file_count):

        #file_count comes from ROS topic published by PC1

        #Check if directory exists, if it doesn't create it
        if not os.path.exists(root_path+'PC2'):
            os.makedirs(root_path+'PC2')

        #Initialize a new CSV file to save PC2 data to
        file_name=root_path+'PC2/Data_PC2_'+str(file_count)+'.csv'
        self.record_filename_pc2=file_name  #Creates a new data csv
        with open(self.record_filename_pc2,'w') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(MOTION_HEADER_PC2)
            file_object.close()

        #Sets the filename (and path) for the left/right ECM video
        self.left_ecm_filename_pc2=root_path+'PC2/LeftECM_PC2_'+str(file_count)+'.mp4'
        self.right_ecm_filename_pc2=root_path+'PC2/RightECM_PC2_'+str(file_count)+'.mp4'

        #initializes the video writer objects
        fourcc=cv2.VideoWriter_fourcc(*'avc1')
        self.left_video_writer=cv2.VideoWriter(self.left_ecm_filename_pc2,fourcc,FRAME_FPS,(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT))
        self.right_video_writer=cv2.VideoWriter(self.right_ecm_filename_pc2,fourcc,FRAME_FPS,(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT))



    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                    str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                    str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                    str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
        
        
        return string_list
    
    def writeRow_PC1(self,task_time,pc1_time,pc2_time,gaze_calib,s_T_psm1,s_T_psm3,cart_T_ecm,ecm_T_psm1,ecm_T_psm3,psm1_joints,psm3_joints,ecm_joints,ecmi_T_ecm,clutch_press,MTMR_Angle,MTML_Angle):
        #PC2 time is published as a ROS topic from PC2

        #s_T_psm1 & s_T_psm3
        if s_T_psm1 is not None:
            s_T_psm1_numpy=np.array(glm.transpose(s_T_psm1).to_list(),dtype='float32')
            s_T_psm1_list=self.convertHomogeneousToCSVROW(s_T_psm1_numpy)
        else:
            s_T_psm1_list=["NaN"]*12

        if s_T_psm3 is not None:
            s_T_psm3_numpy=np.array(glm.transpose(s_T_psm3).to_list(),dtype='float32')
            s_T_psm3_list=self.convertHomogeneousToCSVROW(s_T_psm3_numpy)
        else:
            s_T_psm3_list=["NaN"]*12


        #cart_T_ecm
        if cart_T_ecm is not None:
            cart_T_ecm_numpy=np.array(glm.transpose(cart_T_ecm).to_list(),dtype='float32')
            cart_T_ecm_list=self.convertHomogeneousToCSVROW(cart_T_ecm_numpy)
        else:
            cart_T_ecm_list=["NaN"]*12


        #ecm_T_psm1 & ecm_T_psm3
        if ecm_T_psm1 is not None:
            ecm_T_psm1_numpy=np.array(glm.transpose(ecm_T_psm1).to_list(),dtype='float32')
            ecm_T_psm1_list=self.convertHomogeneousToCSVROW(ecm_T_psm1_numpy)
        else:
            ecm_T_psm1_list=["NaN"]*12

        if ecm_T_psm3 is not None:
            ecm_T_psm3_numpy=np.array(glm.transpose(ecm_T_psm3).to_list(),dtype='float32')
            ecm_T_psm3_list=self.convertHomogeneousToCSVROW(ecm_T_psm3_numpy)
        else:
            ecm_T_psm3_list=["NaN"]*12

        
        #psm1_joints and psm3_joints

        if psm1_joints is not None:
            joint_list_psm1=[str(num) for num in psm1_joints]
        else:
            joint_list_psm1=["NaN"]*7

        if psm3_joints is not None:
            joint_list_psm3=[str(num) for num in psm3_joints]
        else:
            joint_list_psm3=["NaN"]*7

        
        #ecm_joints
        if ecm_joints is not None:
            joint_list_ecm=[str(num) for num in ecm_joints]
        else:
            joint_list_ecm=["NaN"]*4

        if pc2_time is None:
            pc2_time="NaN"

        if ecmi_T_ecm is not None:
            ecmi_T_ecm_numpy=np.array(glm.transpose(ecmi_T_ecm).to_list(),dtype='float32')
            ecmi_T_ecm_list=self.convertHomogeneousToCSVROW(ecmi_T_ecm_numpy)
        else:
            ecmi_T_ecm_list=["NaN"]*12
        #print("in waiting: "+str(self.ser_port.in_waiting))
        if self.arduino_connected:
            if(self.ser_port.in_waiting>0):
                #New data received from arduino
                try:
                    while self.ser_port.in_waiting>0:
                        ser_bytes=self.ser_port.readline()
                    
                    decoded_bytes=ser_bytes.decode("utf-8").strip()
                    if decoded_bytes:
                        arduino_list=decoded_bytes.split(",")
                        arduino_list=[int(item) for item in arduino_list]
                        arduino_list[1]=arduino_list[1]/1000
                        arduino_list[4]=arduino_list[4]/1000
                        arduino_list=[str(item) for item in arduino_list]
                        self.arduino_list=arduino_list  #Updates the arduino list
                    else:
                        print("Empty Arduino Row Received")
                except ValueError as e:
                    print(f"Error processing arduino data: {e}")
                except Exception as e:
                    print(f"Unexpected arduino error: {e}")
        else:
            self.arduino_list=["NaN"]*5

        if clutch_press is None:
            clutch_press="NaN"

        if MTMR_Angle is None:
            MTMR_Angle="NaN"
            
        if MTML_Angle is None:
            MTML_Angle="NaN"




        #pc2_time is already a string
        #Write the row
        row_to_write=[str(task_time),str(pc1_time),pc2_time,str(gaze_calib),""]+s_T_psm1_list+[""]+s_T_psm3_list+[""]+\
        cart_T_ecm_list+[""]+ecm_T_psm1_list+[""]+ecm_T_psm3_list+[""]+joint_list_psm1+[""]+joint_list_psm3+[""]+\
        joint_list_ecm+[""]+ecmi_T_ecm_list+self.arduino_list+[str(clutch_press),str(MTMR_Angle),str(MTML_Angle)]

        with open(self.record_filename_pc1,'a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(row_to_write)
            file_object.close()

    def writeRow_PC2(self,pc2_time,left_ecm_frame_number,right_ecm_frame_number,gaze_frame_number,lc_T_s_numpy,rc_T_s_numpy):

        #lc_T_s & rc_T_s
        if lc_T_s_numpy is not None:
            lc_T_s_list=self.convertHomogeneousToCSVROW(lc_T_s_numpy)
        else:
            lc_T_s_list=["NaN"]*12

        if rc_T_s_numpy is not None:
            rc_T_s_list=self.convertHomogeneousToCSVROW(rc_T_s_numpy)
        else:
            rc_T_s_list=["NaN"]*12

        row_to_write=[str(pc2_time),str(left_ecm_frame_number),str(right_ecm_frame_number),str(gaze_frame_number),""]+lc_T_s_list+[""]+rc_T_s_list
        row_to_write=list(row_to_write)

        with open(self.record_filename_pc2,'a') as file_obj:
            writer_object=csv.writer(file_obj)
            writer_object.writerow(row_to_write)
            file_obj.close()

    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                    str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                    str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                    str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
        
        
        return string_list






    

    





>>>>>>> cb2dc77b51fe54e55eb17648b0794c2400e6f547
