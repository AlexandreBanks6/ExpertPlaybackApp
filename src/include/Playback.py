#Playback: s_T_psm1, s_T_psm3, psm1_joints, and psm3_joints from a csv

import csv
import os
from include import utils
from include import Renderer
import numpy as np
import pandas as pd
import glm

test_point_psm=glm.vec4(0.0,0.0,0.0102,1.0)
mul_mat=glm.mat4x3()  #Selection matrix to take [Xc,Yc,Zc]





class Playback:
    def __init__(self,app):
        self.playback_filename_pc1=None
        self.playback_time_array=None #Array of the recorded task time, will be matched to current playback time
        self.app=app

    def initPlayback(self,rootpath):
        path=rootpath+'PC1/'

        #Check if we have pre-defined playback file, if not take highest-numbered file
        if os.path.isfile(path+self.app.playback_filename):
            self.playback_filename_pc1=path+self.app.playback_filename
        else: #Take highest numbered file
            file_count=1
            file_name=path+'Data_PC1_'+str(file_count)+'.csv'
            while True:
                if os.path.isfile(file_name):
                    file_count+=1
                    file_name=path+'Data_PC1_'+str(file_count)+'.csv'
                else:
                    break
                
            self.playback_filename_pc1=path+'Data_PC1_'+str(file_count-1)+'.csv'

        #Extract the recorded task time, we will then match with the playback time

        with open(self.playback_filename_pc1,'r') as file_object:
            csv_reader=csv.reader(file_object)
            #Skip the first 10 rows because we only care about s_T_psm1, s_T_psm3, psm1_joints, psm3_joints
            for _ in range(10):
                next(csv_reader,None)
            
            #Read the first column
            render_times_list=[float(row[0]) for row in csv_reader if row] #Loops for the rows and returns time as a list of floats
            
            self.playback_time_array=np.array(render_times_list,dtype='float32')
            file_object.close()

    def getDataRow(self):
        index=np.argmin(np.abs(self.playback_time_array-self.app.playback_time)) #Index where current playback time is closest to recorded task time
        
        #print("Playback Time: "+str(self.playback_time_array[index]))
        #Reset the playback_time in the main app to zero if we reached the end of the data
        #We trim 4 positions off the bottom
        if index>=(len(self.playback_time_array)-5):
            self.app.playback_time=0.0000
            index=0

        index=index+10 #Must increment by 10, because playback_time_array ignores first 10 lines
        data_row=pd.read_csv(self.playback_filename_pc1,skiprows=index,nrows=1)
        data_list=data_row.iloc[0].to_list() #Converts the pd row to a list

        return data_list
    
    def getPSM1State(self,data_list):
        #Returns the 4x4 s_T_psm1 glm transform and three joint parameters: psm1_joints=[body rotation (q6), jaw rotation (q7), jaw seperation (theta_j)]

        s_T_psm1_list=data_list[5:17]
        s_T_psm1=self.ConvertDataRow_ToGLMPose(s_T_psm1_list)
        psm1_joints=data_list[74:77]

        return s_T_psm1, psm1_joints
    
    def getPSM3State(self,data_list):
        #Returns the 4x4 s_T_psm3 glm transform and three joint parameters: psm3_joints=[body rotation (q6), jaw rotation (q7), jaw seperation (theta_j)]

        s_T_psm3_list=data_list[18:30]
        s_T_psm3=self.ConvertDataRow_ToGLMPose(s_T_psm3_list)
        psm3_joints=data_list[82:85]

        return s_T_psm3, psm3_joints


    def ConvertDataRow_ToGLMPose(self,data_list):
        transform=glm.mat4(glm.vec4(data_list[3],data_list[6],data_list[9],0),
                           glm.vec4(data_list[4],data_list[7],data_list[10],0),
                           glm.vec4(data_list[5],data_list[8],data_list[11],0),
                           glm.vec4(data_list[0],data_list[1],data_list[2],1))
        return transform
    

    def isSegmentStart(self,seg_index):
        #Function returns true if it is the start of a segment, else it returns false
        start_seg=False
        index=np.argmin(np.abs(self.playback_time_array-self.app.playback_time))
        if index>=seg_index:
            start_seg=True

        return start_seg
    
    def startSegment(self,start_index,end_index,psm_type,c_T_s,cam_mat,dist,mtx):
        #Output: Trajectory pixels (traj_pix) to draw a line of the segment, start_point which defines the start point for the arrow, and end_point which defines end point for arrow
        #Repeat this for left and right cameras
        #Input whether it is PSM1 (right hand) or PSM3 (left hand)

        traj_points_list=[]
        for i in range(start_index,end_index,20):
            index=i+10 #Must increment by 10, because playback_time_array ignores first 10 lines
            data_row=pd.read_csv(self.playback_filename_pc1,skiprows=index,nrows=1)
            data_list=data_row.iloc[0].to_list() #Converts the pd row to a list

            if psm_type=='RIGHT':
                s_T_psm=data_list[5:17]
            else:
                s_T_psm=data_list[18:30]

            s_T_psm=self.ConvertDataRow_ToGLMPose(s_T_psm) #Gets the scene-to-cam transform
            cam_point=c_T_s*s_T_psm*test_point_psm

            proj_point=mul_mat*cam_point
            proj_point=cam_mat*proj_point

            u,v=utils.projectPointsWithDistortion(proj_point[0],proj_point[1],proj_point[2],dist,mtx)
            traj_points_list.append([int(round(u)),int(round(v))])

        
        traj_points=np.array(traj_points_list,dtype=np.int32)
        traj_points=traj_points.reshape((-1,1,2))

        mid_index=len(traj_points)//4
        start_point=tuple(traj_points[mid_index-1][0])
        end_point=tuple(traj_points[mid_index][0])

        return traj_points, start_point, end_point








        
        


