#Class to record robot kinematics/video from PC1 for expert playback app

import moderngl as mgl
import sys
import glm
import numpy as np
import cv2
from datetime import datetime
import csv
import yaml
import os
from include import utils


#File Column Titles:
repeat_string=["Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"] #First number is row of R, second number is column
MOTION_HEADER_PC1=["Task Time","PC1 Time","","PC2 Time","s_T_psm1"]+repeat_string+["s_T_psm3"]+repeat_string+\
    ["cart_T_ecm"]+repeat_string+["ecm_T_psm1"]+repeat_string+["ecm_T_psm3"]+repeat_string+\
            ["psm1_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+["psm3_joints"]+["q1","q2","q3","q4","q5","q6","jaw"]+\
            ["ecm_joints"]+["q1","q2","q3","q4"]

MOTION_HEADER_PC2=["PC2 Time","Frame #","lc_T_s"]+repeat_string+["rc_T_s"]+repeat_string+["is_gaze_calib"]


class DataLogger_PC2:
    def __init__(self,app):
        print('Init Class')
        self.record_filename=None
        self.app=app

    def initRecording_PC1(self,root_path):

        #Initialize a new CSV file to save to
        file_count=1
        file_name=root_path+'Motion_'+str(file_count)+'.csv'
        #Gets a new filename
        while True:
            if os.path.isfile(file_name):
                file_count+=1
                file_name=root_path+'Motion_'+str(file_count)+'.csv'                
            else:
                break
        
        self.record_filename=file_name  #Creates a new motion csv

        #Store all transforms that do not change
        with open(self.record_filename,'w',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(MOTION_HEADER_PC1)

            #Writing si_T_lci (scene to left camera initial)
            writer_object.writerow(['lci_T_si'])
            lci_T_si_numpy=np.array(glm.transpose(self.app.lci_T_si).to_list(),dtype='float32')
            lci_T_si_list=utils.convertHomogeneousToCSVROW(lci_T_si_numpy)
            writer_object.writerow(["",""]+lci_T_si_list)

            #Writing si_T_rci (scene to right camera initial)
            writer_object.writerow(['rci_T_si'])
            rci_T_si_numpy=np.array(glm.transpose(self.app.rci_T_si).to_list(),dtype='float32')
            rci_T_si_list=utils.convertHomogeneousToCSVROW(rci_T_si_numpy)
            writer_object.writerow(["",""]+rci_T_si_list)

            #Writing lc_T_ecm (hand-eye left)
            writer_object.writerow(['ecm_T_lc'])
            ecm_T_lc_numpy=np.array(glm.transpose(self.app.ecm_T_lc).to_list(),dtype='float32')
            ecm_T_lc_list=utils.convertHomogeneousToCSVROW(ecm_T_lc_numpy)
            writer_object.writerow(["",""]+ecm_T_lc_list)

            #Writing rc_T_ecm (hand-eye right)
            writer_object.writerow(['ecm_T_rc'])
            ecm_T_rc_numpy=np.array(glm.transpose(self.app.ecm_T_rc).to_list(),dtype='float32')
            ecm_T_rc_list=utils.convertHomogeneousToCSVROW(ecm_T_rc_numpy)
            writer_object.writerow(["",""]+ecm_T_rc_list)

            #Writing cart_T_ecmi (robot base to ecm initial)
            writer_object.writerow(['cart_T_ecmi'])
            cart_T_ecmi_numpy=np.array(glm.transpose(self.app.cart_T_ecmi).to_list(),dtype='float32')
            cart_T_ecmi_list=utils.convertHomogeneousToCSVROW(cart_T_ecmi_numpy)
            writer_object.writerow(["",""]+cart_T_ecmi_list)

            file_object.close()

    def convertHomogeneousToCSVROW(self,transform):
        #Input: 4x4 numpy array for homogeneous transform
        #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

        string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                    str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                    str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                    str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
        
        
        return string_list
    
    def writeRow_PC1(self,task_time,pc1_time,pc2_time,s_T_psm1,s_T_psm3,cart_T_ecm,ecm_T_psm1,ecm_T_psm3,psm1_joints,psm3_joints,ecm_joints):


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

        
        #Write the row
        row_to_write=[str(task_time),str(pc1_time),str(pc2_time),""]+s_T_psm1_list+[""]+s_T_psm3_list+[""]+\
        cart_T_ecm_list+[""]+ecm_T_psm1_list+[""]+ecm_T_psm3_list+[""]+joint_list_psm1+[""]+joint_list_psm3+[""]+\
        joint_list_ecm

        with open(self.record_filename,'a',newline='') as file_object:
            writer_object=csv.writer(file_object)
            writer_object.writerow(row_to_write)






    

    





