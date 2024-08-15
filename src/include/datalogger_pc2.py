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
MOTION_HEADER=["Time","si_T_psm1"]+repeat_string+["si_T_psm3"]+repeat_string+["si_T_ecm"]+repeat_string+["joint_vars_psm1"]+["q1","q2","q3","q4","q5","q6","jaw"]+["joint_vars_psm3"]+["q1","q2","q3","q4","q5","q6","jaw"]


class DataLogger_PC2:
    def __init__(self,app):
        print('Init Class')
        self.record_filename=None
        self.app=app

    def initRecording(self,root_path):

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
            writer_object.writerow(MOTION_HEADER)

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

            file_object.close()

    





