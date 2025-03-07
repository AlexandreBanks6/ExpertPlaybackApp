<<<<<<< HEAD
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import tkinter as tk
import threading
import os
import yaml
from include import HandEye
from include import Renderer
from scipy.spatial.transform import Rotation

import dvrk
import tf_conversions.posemath as pm
import PyKDL
from include import utils
import cv2.aruco as aruco
#####To Do: Add a functionality to calibrate using an ArUco board, also add functionality to change size of checkerboard

ECM_FRAME_WIDTH_DESIRED=1024
ECM_FRAME_HEIGHT_DESIRED=722

##################Change These Variables If Needed#############################
#These Dimensions should be in meters, should change over to meters
ARUCO_IDs=[4,5,6,7] #List containing the IDs of the aruco markers that we are tracking
NUM_FRAME_DETECTIONS=8 #How many sets of aruco "frames" need to be detected, ToDo for later


CHECKERBOARD_DIM=(8,8) #Number of inner corners in the checkerboard (corners height, corners width)
CHECKER_WIDTH=0.0078232 #Width of each checker in the checkerboard (in meters)

RANSAC_SCENE_REPROJECTION_ERROR=0.0005 #Reprojection error for scene localization RANSAC (in meters)
RANSAC_SCENE_ITERATIONS=1000 #Number of iterations for scene localization RANSAC

#Rigid Body Definition of Ring Over Wire Aruco Holder, each four coordinates define an ArUco marker with corresponding ID:
#Marker corners are defined clockwise from top left
#3D model origin is the top left of AruCO ID 6
ARUCO_SIDELENGTH=0.0254508#0.0253492 #0.025527 #0.0253492 #in meters
ARUCO_SEPERATION=0.10226 #From closest edges, in meters
ARUCO_HEIGHT_OFFSET=0.005 ######!!!!!!!!!!Use Planar ArUco rig!!!!!!!!!!!!!!

RINGOWIRE_MODELPOINTS={
    "6":np.array([
    [0.0,0.0,0.0],
    [ARUCO_SIDELENGTH,0.0,0.0],
    [ARUCO_SIDELENGTH,-ARUCO_SIDELENGTH,0.0],
    [0.0,-ARUCO_SIDELENGTH,0.0]],dtype='float32'),

    "4":np.array([
    [0.0,ARUCO_SEPERATION+ARUCO_SIDELENGTH,ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH,ARUCO_SEPERATION+ARUCO_SIDELENGTH,ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH,ARUCO_SEPERATION,ARUCO_HEIGHT_OFFSET],
    [0.0,ARUCO_SEPERATION,ARUCO_HEIGHT_OFFSET]],dtype='float32'),

    "5":np.array([
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,2*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,2*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,2*ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,2*ARUCO_HEIGHT_OFFSET]],dtype='float32'),

    "7":np.array([
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,3*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,3*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SIDELENGTH,3*ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SIDELENGTH,3*ARUCO_HEIGHT_OFFSET]],dtype='float32')
}


FRAMES_TO_REMOVE_RIGHT=[22,28]   #These frames were determined to be garbage visually
FRAMES_TO_REMOVE_LEFT=[22,26,28,31]
NUM_FRAMES_CAPTURED=41

REQUIRED_CHECKERBOARD_NUM=10 #Number of checkerboard images needed for the calibration
ERROR_THRESHOLD=10 #Pixel Error Threshold for centering ECM above checkerboard

#PROPORTIONAL_GAIN=20 #Gain for centering ECM above checkerboard
##MAX_ITERATIONS=40
#Where the images for the calibration are saved

CALIBRATION_DIR="../resources/Calib/" #Where we store calibration parameters and images


RIGHT_FRAMES_FILEDIR='/chessboard_images_right/'
LEFT_FRAMES_FILEDIR='/chessboard_images_left/'

#Where the calibration parameters are saved
RIGHT_CAMERA_CALIB_DIR="/calibration_params_right/"
LEFT_CAMERA_CALIB_DIR="/calibration_params_left/"

#Where we store the base_T_ecm poses
BASE_TO_ECM_DIR="/base_T_ecm_poses/"

#Ros Topics
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'


# Preset Motions from Starting Pose

'''
Sets of 7 motions for 3 different z axis increments:
Firt, increment z
then:
1. No rotation/Translation
2. Rotate about z by positive angle
3. Rotate about z by negative angle
4. Rotate about x by positive angle
5. Rotate about x by negative angle
6. Translate along x by positive val
7. translate along y by positive val
Repeat 3 times

We define these as numpy frames, they are later converted to PyKDL frames

'''

z_translation=0.0075 #Amount that we translate along z (1.5 centimeters)
Z_MOTION=[0,z_translation,z_translation*2,z_translation*3]
planar_translation=0.005 #Amount that we translate in plane parallel to endoscope
rotation_angle=2*(np.pi/180) #20 degrees in Rad

#T1, no change
T1=np.identity(4)
print("T1: "+str(T1))
#T2 rotation about z positive
Rz=utils.rotationZ(rotation_angle)
Rz=utils.EnforceOrthogonalityNumpy(Rz)
T2=np.identity(4)
T2[0:3,0:3]=Rz
print("T2: "+str(T2))
#T3 rotation about z negative
Rz=utils.rotationZ(-rotation_angle)
Rz=utils.EnforceOrthogonalityNumpy(Rz)
T3=np.identity(4)
T3[0:3,0:3]=Rz
print("T3: "+str(T3))

#T4 rotation about x positive
Rx=utils.rotationX(rotation_angle/2)
Rx=utils.EnforceOrthogonalityNumpy(Rx)
T4=np.identity(4)
T4[0:3,0:3]=Rx
print("T2: "+str(T4))
#T5 rotation about x negative
Rx=utils.rotationX(-rotation_angle/2)
Rx=utils.EnforceOrthogonalityNumpy(Rx)
T5=np.identity(4)
T5[0:3,0:3]=Rx
print("T3: "+str(T5))

#T4 translation along x
T6=np.identity(4)
T6[0,3]=planar_translation
print("T4: "+str(T6))
#T4 translation along y
T7=np.identity(4)
T7[1,3]=planar_translation
print("T5: "+str(T7))
MOTIONS=[T1,T2,T3,T4,T5,T6,T7]



class CameraCalibGUI:

    def __init__(self):

        self.bridge=CvBridge()

        self.rootName=None #Root folder that we are writing calibration parameters to

        #Counters
        self.ranOnce=0
        self.frame_number=0


        ######Aruco Things:
        self.aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_1000) #using the 4x4 dictionary to find markers
        self.aruco_params=aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshConstant=10



        #####################Setting Up GUI#####################
        self.window=tk.Tk()
        self.window.title("dVRK Camera Calibrator")
        self.window.rowconfigure([0,1,2,3,4],weight=1)
        self.window.columnconfigure([0,1],weight=1)

        #----Message Box
        self.message_label=tk.Label(text="Welcome to the dVRK Endoscope Calibrator",width=40)
        self.message_label.grid(row=0,column=0)
        self.message_label2=tk.Label(text="Performs Camera Calibration and Hand-Eye Calibration",width=60)
        self.message_label2.grid(row=1,column=0)

        #---Buttons

        #The grab frame callback automatically moves the robot to a new pose an grabs a frame
        self.button_frame=tk.Button(text="Grab Frame(s)",width=15,command=self.grabFramesCallback)
        self.button_frame.grid(row=2,column=0,sticky="nsew")

        self.button_savemanual=tk.Button(text="Save Manual",width=15,command=self.saveManualCallback)
        self.button_savemanual.grid(row=2,column=1,sticky="nsew")

        self.button_calibrate=tk.Button(text="Calibrate",width=15,command=self.calibrateCameraCallback)
        self.button_calibrate.grid(row=3,column=0,sticky="nsew")
        
        #Tick Box for Manual camera calibration
        self.checkbox_manual=tk.Checkbutton(text="Manual",onvalue=1,offvalue=0,command=self.manualCheckbox)
        self.checkbox_manual.grid(row=3,column=1,sticky='nsew')



        self.calbration_message_label=tk.Label(text="",width=60)
        self.calbration_message_label.grid(row=4,column=0)


        #Frames that we are reading from endoscope
        self.frameLeft=None
        self.frameRight=None
        self.isManual=False

        #Subscribing to ROS topics
        rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackRight,queue_size=1,buff_size=2**18)
        rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackLeft,queue_size=1,buff_size=2**18)
        
        #Setting up the ecm
        self.ecm=dvrk.ecm("ECM")
        enable_true=self.ecm.enable()
        home_true=self.ecm.home()
        print("Enable: "+str(enable_true))
        print("Home: "+str(home_true))



        #Checkerboard subpix criteria:
        self.criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 0.001)


        


        #Execution Loop  
        #Window where we display the left/right frames:
        #cv2.namedWindow("Left/Right Frames",cv2.WINDOW_NORMAL)
        



        #Hand-Eye Parameteres
        self.hand_eye=HandEye.HandEye() #Creates hand-eye calibration object
        self.rb_T_ecm_list=[]   #List of robot base (cart) to ecm transforms
        
        self.mtx_right=None
        self.mtx_left=None
        self.dst_right=None
        self.dist_left=None
        self.rb_T_ecm_list=[]
        self.rb_T_ecm_setpoint_list=[]

        rospy.sleep(1)

        self.window.after(1,self.showFramesCallback)
        self.window.mainloop()
    
    def manualCheckbox(self):
        self.isManual=not self.isManual

    def showFramesCallback(self):

        if self.ranOnce>=2:
            #print("Entered")
            frameLeft=cv2.resize(self.frameLeft,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
            frameRight=cv2.resize(self.frameRight,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Left Frame",frameLeft)
            cv2.imshow("Right Frame",frameRight)
            # img_combined=cv2.hconcat([self.frameLeft,self.frameRight])
            # img_combined=cv2.resize(img_combined,(361,512),interpolation=cv2.INTER_LINEAR)
            # cv2.imshow("Left/Right Frames",img_combined)
            cv2.waitKey(1)
        self.window.after(1,self.showFramesCallback)
    
    #Callback for grabbing frames
    def frameCallbackRight(self,data):
        self.frameRight=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.frameRight=cv2.resize(self.frameRight,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
        self.ranOnce+=1

    def frameCallbackLeft(self,data):
        self.frameLeft=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.frameLeft=cv2.resize(self.frameLeft,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
        self.ranOnce+=1
        #print("New Frame")
    def findArUcos(self,frame):
        frame_gray=cv2.cvtColor(frame.copy(),cv2.COLOR_BGR2GRAY)
        corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)
        #corners=cv2.cornerSubPix(frame_gray,corners,(11,11),(-1,-1),self.criteria)
        corners_filtered=[]
        ids_filtered=[]
        if ids is not None:
            for id,corner in zip(ids,corners):
                if id[0] in ARUCO_IDs:
                    corners_filtered.append(corner)
                    ids_filtered.append([id[0]])
            ids=np.array(ids_filtered,dtype='float32')
            corners=np.array(corners_filtered,dtype='float32')
            if len(ids_filtered)>0:
                return ids,corners
            else:
                self.calbration_message_label.config(text="ArUco Out of View. Make sure to position ECM far enough above")
                return None,None
        else:
            self.calbration_message_label.config(text="ArUco Out of View. Make sure to position ECM far enough above")
            return None,None
    def findAxisErrors(self,frame,ids,corners):
        
        image_center=[frame.shape[1]/2,frame.shape[0]/2]
        corners_for_mean=None
        for corner in corners:
            if corners_for_mean is None:
                corners_for_mean=corner[0]
            else:
                corners_for_mean=np.vstack((corners_for_mean,corner[0]))
        #print("Corners for mean: "+str(corners_for_mean))
        checkerboard_center=np.mean(corners_for_mean,axis=0)
        #print("checkerboard_center: "+str(checkerboard_center))
        e_x=image_center[0]-checkerboard_center[0]
        e_y=image_center[1]-checkerboard_center[1]
        #Displaying the Checkerboard Center (red) and Camera Center (green)
        new_frame=frame
        new_frame=cv2.circle(new_frame,(int(checkerboard_center[0]),int(checkerboard_center[1])),3,(0,0,255),3)  #Drawing the checkerboard center
        new_frame=cv2.circle(new_frame,(int(image_center[0]),int(image_center[1])),ERROR_THRESHOLD,(0,255,0),2)  #Drawing the Camera Center
        new_frame=aruco.drawDetectedMarkers(frame,corners=corners,ids=ids)
        cv2.imshow('Alignment Error', new_frame)
        cv2.waitKey(50)
        return e_x,e_y

    def saveManualCallback(self):

        os.mkdir(self.rootName+BASE_TO_ECM_DIR)
        rb_T_ecm_store=np.array(self.rb_T_ecm_list,dtype='float32')
        print("rb_T_ecm_store: "+str(rb_T_ecm_store))
        np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm',rb_T_ecm_store)

        rb_T_ecm_setpoint_store=np.array(self.rb_T_ecm_setpoint_list,dtype='float32')
        np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm_setpoint',rb_T_ecm_setpoint_store)

        self.calbration_message_label.config(text="Frame Grabbing Done")

        self.rb_T_ecm_list=[]
        self.rb_T_ecm_setpoint_list=[]
        self.frame_number=0



    def grabFramesCallback(self):
        frameLeft=cv2.resize(self.frameLeft,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
        frameRight=cv2.resize(self.frameRight,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)

        if self.isManual:
            #Manually grab the frames
            if self.frame_number==0:
                #Initialize the save folder
                self.createSaveFolder()
                self.rb_T_ecm_list=[]
            
            file_name_right=self.rootName+RIGHT_FRAMES_FILEDIR
            file_name_left=self.rootName+LEFT_FRAMES_FILEDIR     
            if not os.path.isdir(file_name_right):
                os.mkdir(file_name_right)
                os.mkdir(file_name_left)
            
            ecm_pose=self.ecm.measured_cp()
            rb_T_ecm=pm.toMatrix(ecm_pose)
            self.rb_T_ecm_list.append(rb_T_ecm)

            ecm_pose=self.ecm.setpoint_cp()
            rb_T_ecm=pm.toMatrix(ecm_pose)
            self.rb_T_ecm_setpoint_list.append(rb_T_ecm)


            cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",frameRight)
            cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",frameLeft)

            self.frame_number+=1
            self.calbration_message_label.config(text="Frame Count: "+str(self.frame_number))
        else:


            #Automatically grab the frames and save them

            self.just_grabbed_frames=True
            self.createSaveFolder()    #This Creates a new folder with an incremented folder number (Calib_num)
            file_name_right=self.rootName+RIGHT_FRAMES_FILEDIR
            file_name_left=self.rootName+LEFT_FRAMES_FILEDIR

            #First we get user to align the ECM center with the checkerboard center
            self.calbration_message_label.config(text="Move ArUco Object Center (red) to Camera Center (green)")
            err_mag=ERROR_THRESHOLD+1
            print("Entered Grabbed Frames")
            while err_mag>ERROR_THRESHOLD: #Loops until center of LEFT ECM is within acceptable threshold
                ids,corners=self.findArUcos(self.frameLeft)
                if corners is not None:
                    e_x,e_y=self.findAxisErrors(self.frameLeft,ids,corners)
                    err_mag=np.sqrt((e_x**2)+(e_y**2))
                    print("err_mag: "+str(err_mag))

            print("Aligned")
            cv2.destroyWindow('Alignment Error')
            rospy.sleep(5)  #Sleep to allow user to move away from ecm

            #Now we capture the frames for the camera calibration
            if not os.path.isdir(file_name_right):
                os.mkdir(file_name_right)
                os.mkdir(file_name_left)
            print("File Name: "+str(file_name_right))
            self.frame_number=0
            #We loop through and move the ECM, then take frames, we also get the rb_T_ecm transform
            self.rb_T_ecm_list=[]
            translation_sign=-1
            
            ecm_pose_init=self.ecm.measured_cp()
            print("measured_cp")
            for i in range(len(Z_MOTION)): #Loop for 3 z values
                translation_sign=-1*translation_sign
                for j in range(len(MOTIONS)): #Loop for motions in this plane                
                    #Move it back to central location before translating again
                    print("starting move")
                    self.ecm.move_cp(ecm_pose_init).wait()
                    print("moving")
                    rospy.sleep(0.7)
                    ecm_pose_curr=self.ecm.measured_cp()
                    print("moved")

                    #print("ECM Position="+str(ecm_pose_curr.p))
                    #print("ECM Rotation="+str(ecm_pose_curr.M))
                    #print("z motion: "+str(Z_MOTION[i]))
                    #print("Tranform: "+str(MOTIONS[j]))
                    
                    motion_mat=pm.fromMatrix(MOTIONS[j])
                    
                    ecm_pose_curr.p[0]+=translation_sign*motion_mat.p[0]
                    ecm_pose_curr.p[1]+=translation_sign*motion_mat.p[1]
                    ecm_pose_curr.p[2]+=Z_MOTION[i] #increments z pose
                    
                    ecm_pose_curr.M=ecm_pose_curr.M*motion_mat.M
                    self.ecm.move_cp(ecm_pose_curr).wait()
                    rospy.sleep(1.5)                

                    #Grab the pose of ecm w.r.t. robot base
                    ecm_pose_new=self.ecm.measured_cp()
                    print("ecm_pose: "+str(ecm_pose_new))
                    rb_T_ecm=pm.toMatrix(ecm_pose_new)
                    print("ecm_pose numpy: "+str(rb_T_ecm))
                    self.rb_T_ecm_list.append(rb_T_ecm)

                    cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",self.frameRight)
                    cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",self.frameLeft)

                    self.frame_number+=1
            print("Num Frames: "+str(self.frame_number))
            #Store the rb_T_ecm poses in a yaml file
            os.mkdir(self.rootName+BASE_TO_ECM_DIR)
            rb_T_ecm_store=np.array(self.rb_T_ecm_list,dtype='float32')
            print("rb_T_ecm_store: "+str(rb_T_ecm_store))
            np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm',rb_T_ecm_store)

            rb_T_ecm_setpoint_store=np.array(self.rb_T_ecm_setpoint_list,dtype='float32')
            np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm_setpoint',rb_T_ecm_setpoint_store)

            self.calbration_message_label.config(text="Frame Grabbing Done")

                

 

        #First, servo ECM in 2D (along x,y plane) to have ECM center at checkerboard center (there is no z-axis movement)
        #For now only servo based on left camera
        #corners=self.findCheckerboard(self.frameLeft)
        #if corners is not None:
            #Get the rotation between 2x2 coordinate systems
        '''
        checkerboard_center_initial=np.mean(corners,axis=0) #Initial checkerboard center
        delta=np.array([0.04,0.04]) #How much we move ECM by (direction of how we expect it to move in camera coord system)
        ecm_pose_curr=self.ecm.setpoint_cp()
        ecm_pose_curr.p[0]+=delta[0]
        ecm_pose_curr.p[1]+=delta[1]
        self.ecm.move_cp(ecm_pose_curr).wait()
        rospy.sleep(0.05)
        corners=self.findCheckerboard(self.frameLeft)
        checkerboard_center_actual=np.mean(corners,axis=0)
        delta_actual=np.array([checkerboard_center_actual[0][0]-checkerboard_center_initial[0][0],checkerboard_center_actual[0][1]-checkerboard_center_initial[0][1]])

        t_hat=delta/np.sqrt((delta[0]**2)+(delta[1]**2))
        s_hat=delta_actual/np.sqrt((delta_actual[0]**2)+(delta_actual[1]**2))

        if np.linalg.norm(s_hat+t_hat)<10**(-6):
            alpha_mag=2*np.arctan(np.linalg.norm(s_hat-t_hat)/np.linalg.norm(s_hat+t_hat+10**(-4)))

        else:
            alpha_mag=2*np.arctan(np.linalg.norm(s_hat-t_hat)/np.linalg.norm(s_hat+t_hat))
        cross_prod=np.cross(t_hat,s_hat)
        print("Alpha Mag: "+str(alpha_mag*(180/np.pi)))
        alpha=np.sign(cross_prod)*alpha_mag
        theta_z=alpha
        print("Rotation Angle: "+str(theta_z*(180/np.pi)))
        Rz=np.array([
            [np.cos(theta_z),-np.sin(theta_z)],
            [np.sin(theta_z),np.cos(theta_z)]
        ])

        err_mag=ERROR_THRESHOLD+1
        '''
        '''
        #Finds mapping from change in camera coord system (pixels) to change in ECM coord system (meters) using regression
        move_ecm_list=[[0.0025,0.0025],[-0.005,0],[0,-0.005],[0.005,0.0025],[0,0.005],[0.0065,0],[-0.0055,-0.0045]]  #Set of 8 movements to write to ECM
        #move_ecm_list=[[0.01,0.01],[0.01,0],[0,0.01]]
        delta_ecm_list=[]
        delta_cam_list=[]
        corners=self.findCheckerboard(self.frameLeft)
        if corners is not None:
            checkerboard_center_initial=np.mean(corners,axis=0) #Initial checkerboard center
            ecm_pose_curr=self.ecm.measured_cp()
            for i in range(0,len(move_ecm_list)):  
                print("Move #: "+str(i))
                print("Move x: "+str(move_ecm_list[i][0])) 
                print("Move y: "+str(move_ecm_list[i][1]))               
                
                ecm_pose_curr.p[0]+=move_ecm_list[i][0]
                ecm_pose_curr.p[1]+=move_ecm_list[i][1]
                #self.ecm.move_cp(ecm_pose_curr)
                self.ecm.move_cp(ecm_pose_curr).wait()
                rospy.sleep(1)
                ecm_pose_new=self.ecm.measured_cp()

                delta_ecm_list.append([ecm_pose_new.p[0]-ecm_pose_curr.p[0],ecm_pose_new.p[1]-ecm_pose_curr.p[1]])
                print('Delta ECM List: '+str(delta_ecm_list))
                ecm_pose_curr=ecm_pose_new
            
                corners=self.findCheckerboard(self.frameLeft)
                if corners is not None:
                    checkerboard_center_new=np.mean(corners,axis=0)
                    delta_cam=[checkerboard_center_new[0][0]-checkerboard_center_initial[0][0],checkerboard_center_new[0][1]-checkerboard_center_initial[0][1]]
                    print("delta_cam: "+str(delta_cam))
                    delta_cam_list.append(delta_cam)
                    checkerboard_center_initial=checkerboard_center_new
                else:
                    print("Returned")
                    return  
            #Now we run the regression to get the transformation A such that: Ax=b 
            #where x=delta_cam and b=delta_ecm
            x_observations=np.array(delta_cam_list)
            b_observations=np.array(delta_ecm_list)
            A,residuals,rank,s=np.linalg.lstsq(x_observations,b_observations,rcond=None)
            print("Estimated A:"+str(A))
            print("Residual:"+str(residuals))

            
            err_mag=ERROR_THRESHOLD+1
            loop_count=0
            while err_mag>ERROR_THRESHOLD and loop_count<MAX_ITERATIONS: #Loops until center of LEFT ECM is within acceptable threshold
                corners=self.findCheckerboard(self.frameLeft)
                if corners is not None:
                    e_x,e_y=self.findAxisErrors(self.frameLeft,corners)
                    print("e_x,e_y: "+str([-e_x,-e_y]))

                    err_mag=np.sqrt((e_x**2)+(e_y**2))
                    error_vec=np.array([e_x,e_y]) #This is delta_cam
                    error_vec=PROPORTIONAL_GAIN*error_vec
                    print("error_vec: "+str(error_vec))
                    #move=PROPORTIONAL_GAIN*np.dot(Rz,error_vec)
                    delta_ecm=np.dot(A,error_vec) #A @ error_vec
                    print("move: "+str(delta_ecm))
                    ecm_pose_curr=self.ecm.measured_cp()
                    ecm_pose_curr.p[0]+=delta_ecm[0]
                    ecm_pose_curr.p[1]+=delta_ecm[1]
                    self.ecm.move_cp(ecm_pose_curr).wait()
                    #self.ecm.servo_cp(ecm_pose_curr)
                    rospy.sleep(0.05)
                    loop_count+=1


                else:
                    print("Returned")
                    return    

            print("Got to Middle")
            print("Grabbing Frames Starting")
            #Now we capture the frames for the camera calibration
            if not os.path.isdir(file_name_right):
                os.mkdir(file_name_right)
                os.mkdir(file_name_left)
            print("File Name: "+str(file_name_right))
            self.frame_number=1
            #We loop through and move the ECM, then take frames:
            for i in range(0,len(Z_MOTION)): #Loop for 3 z values
                for j in range(0,len(MOTIONS)): #Loop for motions in this plane
                    print("z motion: "+str(Z_MOTION[i]))
                    #print("motion: "+str(MOTIONS[j]))
                    ecm_pose_curr=self.ecm.measured_cp()
                    print("ECM Pose="+ecm_pose_curr.p)
                    ecm_pose_curr.p[2]+=Z_MOTION[i] #increments z pose

                    #ecm_pose_curr=ecm_pose_curr*pm.fromMatrix(MOTIONS[j])
                    self.ecm.move_cp(ecm_pose_curr).wait()
                    rospy.sleep(1)
                    cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",self.frameRight)
                    cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",self.frameLeft)
                    self.frame_number+=1
                    self.calibration_count_label.config(text="# of frames="+str(self.frame_number))


        else:
            print("Returned")
            return

        '''


        '''
        if not os.path.isdir(file_name_right):
            os.mkdir(file_name_right)
            os.mkdir(file_name_left)
        cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",self.frameRight)
        cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",self.frameLeft)
        self.frame_number+=1
        self.calibration_count_label.config(text="# of frames="+str(self.frame_number))
        '''
   
    def rightCheckbox(self):
        CameraCalibGUI.isRight=not CameraCalibGUI.isRight
        print("Callback")

    def leftCheckbox(self):
        CameraCalibGUI.isLeft=not CameraCalibGUI.isLeft


    def calibrateCameraCallback(self):
        #Does Both the Right and Left Cameras hand-eye+general calibration (using ArUco Markers)

        self.getFolderName()    #Gets Most Recent File Directory
        print("Root Name: "+self.rootName)
        frames_right_path=self.rootName+RIGHT_FRAMES_FILEDIR
        print("frames_right_path: "+frames_right_path)
        frames_left_path=self.rootName+LEFT_FRAMES_FILEDIR
        print("frames_left_path: "+frames_left_path)
        calibration_params_right=self.rootName+RIGHT_CAMERA_CALIB_DIR
        print("calibration_params_right: "+calibration_params_right)
        calibration_params_left=self.rootName+LEFT_CAMERA_CALIB_DIR
        print("calibration_params_left: "+calibration_params_left)
        frames_path=[frames_right_path,frames_left_path]
        params_path=[calibration_params_right,calibration_params_left]

        #################Camera Calibration###############
        #Repeats Twice, right first then left
        
        for i in range(2):
            calibration_params_path=params_path[i]
            checkerboard_frames_path=frames_path[i]

            if not os.path.isdir(calibration_params_path):
                os.mkdir(calibration_params_path)

            #Camera Calibration params
            objp = np.zeros((CHECKERBOARD_DIM[0]*CHECKERBOARD_DIM[1],3), np.float32)
            objp[:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)*CHECKER_WIDTH  #I think this is already the object point size as it should be

            objpoints=[]
            imgpoints=[]
            found=0
            for file in os.listdir(checkerboard_frames_path):
                filename=os.fsdecode(file)
                if filename.endswith(".jpg"):
                    path=checkerboard_frames_path+filename
                    img = cv2.imread(path) # Capture frame-by-frame
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    # Find the chess board corners
                    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_DIM, None)

                    # If found, add object points, image points (after refining them)
                    if ret == True:
                        objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
                        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
                        imgpoints.append(corners2)
                        # Draw and display the corners
                        img = cv2.drawChessboardCorners(img, CHECKERBOARD_DIM, corners2, ret)
                        found += 1
                        cv2.imshow('Chessboard Frame', img)
                        cv2.waitKey(5)
            
            if found>=REQUIRED_CHECKERBOARD_NUM:
                cv2.destroyWindow('Chessboard Frame')
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
                if i==0:
                    self.mtx_right=mtx
                    self.dst_right=dist
                if i==1:
                    self.mtx_left=mtx
                    self.dst_left=dist
            else:
                self.calbration_message_label.config(text="Note enough acceptable checkerboards, grab more frames, need "+str(REQUIRED_CHECKERBOARD_NUM))
                return
            
            #Finding Re-projection error
            error_list=[]
            for j in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[j], rvecs[j], tvecs[j], mtx, dist)
                error = cv2.norm(imgpoints[j], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                error_list.append(error)
            print("error list: "+str(error_list))
            error_list=np.array(error_list,dtype='float32')
            data = {'camera_matrix': np.asarray(mtx).tolist(),
                        'dist_coeff': np.asarray(dist).tolist(),
                        'mean reprojection error': float(np.mean(error_list)),
                        'std reprojection error': float(np.std(error_list))}
            
            #Writing data to file
            with open(calibration_params_path+"calibration_matrix.yaml","w") as f:
                yaml.dump(data,f)

        print("Finished Camera Calibration")




        ####################Hand Eye Calibration######################
        '''
        General Algorithm:
        Loop through and get each checkerboard, if it exists store pose of scene_T_camera and ecm_T_psm3 (do this for both left and right)
        Then convert these to the A (ecm poses) and B matrices (camera poses)
        '''
        rb_T_ecm_right=[]
        rb_T_ecm_left=[]

        rightcam_T_scene=[]
        leftcam_T_scene=[]

        right_cam_T_scene_rvec=[]
        right_cam_T_scene_tvec=[]

        left_cam_T_scene_rvec=[]
        left_cam_T_scene_tvec=[]

        #Open Up the rb_T_ecm list
        rb_T_ecm_path=self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm_setpoint.npy'
        #print("rb_T_ecm_path: "+str(rb_T_ecm_path))
        rb_T_ecm_list=np.load(rb_T_ecm_path)

        #print("rb_T_ecm_list: "+str(rb_T_ecm_list))
        #Loop twice, right first then left
        for i in range(2):
            calibration_params_path=params_path[i]
            checkerboard_frames_path=frames_path[i]
            if i==0:
                #print("i="+str(i))
                mtx=self.mtx_right
                
                dist=self.dst_right
                frame_name='frame_right'
                #print("frame="+frame_name)
            elif i==1:
                #print("i="+str(i))
                frame_name='frame_left'
                mtx=self.mtx_left
                dist=self.dst_left
                #print("frame="+frame_name)
            
            #Loops for each ArUco that we previously captured
            for frame_num in range(NUM_FRAMES_CAPTURED):
                #print('frame_num: '+str(frame_num))
                filename=checkerboard_frames_path+frame_name+str(frame_num)+".jpg"
                img = cv2.imread(filename) # Capture frame-by-frame
                frame_gray=cv2.cvtColor(img.copy(),cv2.COLOR_BGR2GRAY)
                corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)
                


                if ids is not None: #We found IDs
                    '''
                    corners=np.array(list(corners),dtype='float32')
                    corners=corners.reshape(-1,4,2)
                    dim1=corners.shape[0]
                    dim2=corners.shape[1]
                    corners=corners.reshape(dim1*dim2,2)
                    corners=cv2.cornerSubPix(frame_gray,corners,(5,5),(-1,-1),self.criteria)                    
                    corners=corners.reshape(dim1,dim2,2)                    
                    corners=corners.reshape(dim1,1,dim2,2)                    
                    corners=tuple(corners)
                    '''


                    #Keeps only the ids that we want
                    corners_filtered=[]
                    ids_filtered=[]
                    for id,corner in zip(ids,corners):
                        if id[0] in ARUCO_IDs:
                            corners_filtered.append(corner)
                            ids_filtered.append([id[0]])


                    image_points=None
                    model_points=None
                    if len(ids_filtered)>0: #We found IDs after filtering
                        ids_print=np.array(ids_filtered)
                        corners_print=np.array(corners_filtered,dtype='float32')
                        #Display the corners:
                        frame_converted=aruco.drawDetectedMarkers(img,corners=corners_print,ids=ids_print)
                        #cv2.imshow('ArUco Frame', frame_converted)
                        #cv2.waitKey(500)
                        #Now we append the corresponding model and object points to these two arrays:
                        for id,corner in zip(ids_filtered,corners_filtered):
                            if image_points is None:
                                image_points=corner[0]
                                model_points=RINGOWIRE_MODELPOINTS[str(id[0])]
                            else:
                                image_points=np.vstack((image_points,corner[0]))
                                model_points=np.vstack((model_points,RINGOWIRE_MODELPOINTS[str(id[0])]))
                        #print("image_points"+str(image_points))
                        #print("model_points"+str(model_points))
                        #print("ids_filtered"+str(ids_filtered))

                        success,rotation_vector,translation_vector,_=cv2.solvePnPRansac(model_points,image_points,mtx,dist,\
                                                                                        iterationsCount=RANSAC_SCENE_ITERATIONS,reprojectionError=RANSAC_SCENE_REPROJECTION_ERROR,flags=cv2.USAC_MAGSAC)
                        

                        #Convert rotation_vector and translation_vector to homogeneous transform
                        if success:
                            frame_converted=img.copy()
                            frame_converted=cv2.drawFrameAxes(frame_converted,mtx,\
                                      dist,rotation_vector,translation_vector,0.05)
                            cv2.imshow('Pose',frame_converted)
                            cv2.waitKey(1)
                            #print("rotation_vector: "+str(rotation_vector))
                            #print("translation_vector: "+str(translation_vector))
                            cam_T_scene=utils.convertRvecTvectoHomo(rotation_vector,translation_vector)
                            #cam_T_scene=utils.invHomogeneousNumpy(cam_T_scene)
                            #cam_T_scene=utils.invHomogeneousNumpy(cam_T_scene)
                            #cam_T_scene=utils.EnforceOrthogonalityNumpy_FullTransform(cam_T_scene)

                            rb_T_ecm=rb_T_ecm_list[frame_num]
                            
                            rb_T_ecm=utils.EnforceOrthogonalityNumpy_FullTransform(rb_T_ecm)
                            #print('rb_T_ecm: '+str(rb_T_ecm))
                            #rb_T_ecm=utils.EnforceOrthogonalityNumpy_FullTransform(rb_T_ecm)

                            #Updating the lists
                            if i==0: #right
                                rb_T_ecm_right.append(rb_T_ecm)
                                rightcam_T_scene.append(cam_T_scene)
                                right_cam_T_scene_rvec.append(rotation_vector)
                                right_cam_T_scene_tvec.append(translation_vector)
                            elif i==1:
                                rb_T_ecm_left.append(rb_T_ecm)
                                leftcam_T_scene.append(cam_T_scene)
                                left_cam_T_scene_rvec.append(rotation_vector)
                                left_cam_T_scene_tvec.append(translation_vector)


        #Filter out bad frames in left and right
        rb_T_ecm_right_new=[]
        rightcam_T_scene_new=[]
        right_cam_T_scene_rvec_new=[]
        right_cam_T_scene_tvec_new=[]

        for i in range(len(rb_T_ecm_right)):
            if i not in FRAMES_TO_REMOVE_RIGHT:
                #print("i: "+str(i))
                rb_T_ecm_right_new.append(rb_T_ecm_right[i])
                rightcam_T_scene_new.append(rightcam_T_scene[i])
                right_cam_T_scene_rvec_new.append(right_cam_T_scene_rvec[i])
                right_cam_T_scene_tvec_new.append(right_cam_T_scene_tvec[i])

        rb_T_ecm_right=rb_T_ecm_right_new
        rightcam_T_scene=rightcam_T_scene_new

        left_cam_T_scene_rvec_new=[]
        left_cam_T_scene_tvec_new=[]
        rb_T_ecm_left_new=[]
        leftcam_T_scene_new=[]
        for i in range(len(rb_T_ecm_left)):
            if i not in FRAMES_TO_REMOVE_LEFT:
                rb_T_ecm_left_new.append(rb_T_ecm_left[i])
                leftcam_T_scene_new.append(leftcam_T_scene[i])
                left_cam_T_scene_rvec_new.append(left_cam_T_scene_rvec[i])
                left_cam_T_scene_tvec_new.append(left_cam_T_scene_tvec[i])
        rb_T_ecm_left=rb_T_ecm_left_new
        leftcam_T_scene=leftcam_T_scene_new



        #Solving Hand-Eye Calibration for both the right and left cameras (for now just use the OpenCV method)
        
        #Do Right First
        A=[]
        B=[]
        cv2.destroyWindow('Pose')
        R_gripper2base=[]
        t_gripper2base=[]
        R_target2cam=[]
        t_target2cam=[]


        #for i in range(len(rb_T_ecm_right)-1):
        for i in range(len(rb_T_ecm_right)-1):

            #A_i=np.dot(utils.invHomogeneousNumpy(rightcam_T_scene[i+1]),rightcam_T_scene[i]) #Initial
            A_i=rightcam_T_scene[i]@utils.invHomogeneousNumpy(rightcam_T_scene[i+1]) #Best
            A_i=utils.EnforceOrthogonalityNumpy_FullTransform(A_i)
            #print("A_i: "+str(A_i))
            #B_i=np.dot(rb_T_ecm_right[i+1],utils.invHomogeneousNumpy(rb_T_ecm_right[i])) #Initial

            B_i=utils.invHomogeneousNumpy(rb_T_ecm_right[i])@rb_T_ecm_right[i+1] #Best
            B_i=utils.EnforceOrthogonalityNumpy_FullTransform(B_i)

            A.append(B_i)
            B.append(A_i)
            #print("B_i: "+str(B_i))

            # R_gripper2base.append(rb_T_ecm_right[i][0:3,0:3])
            # t_gripper2base.append(rb_T_ecm_right[i][0:3,3])
            
            # R_target2cam.append(right_cam_T_scene_rvec_new[i])
            # t_target2cam.append(right_cam_T_scene_tvec_new[i])

            R_gripper2base.append(rb_T_ecm_right[i][0:3,0:3])
            t_gripper2base.append(rb_T_ecm_right[i][0:3,3])

            R_target2cam.append(rightcam_T_scene[i][0:3,0:3])
            t_target2cam.append(rightcam_T_scene[i][0:3,3])


        
        #Solve hand-eye problem
        A=np.array(A,dtype='float32')
        B=np.array(B,dtype='float32')
        R_gripper2base=np.array(R_gripper2base,dtype='float32')
        t_gripper2base=np.array(t_gripper2base,dtype='float32')
        R_target2cam=np.array(R_target2cam,dtype='float32')
        t_target2cam=np.array(t_target2cam,dtype='float32')
        
        ecm_T_rightcam=self.hand_eye.ComputeHandEye(A,B)
        #print("ecm_T_rightcam: "+str(ecm_T_rightcam))
        #data_right = {'ecm_T_rightcam': ecm_T_rightcam.tolist()}

        #print("R_gripper2base: "+str(R_gripper2base))
        #print("t_gripper2base: "+str(t_gripper2base))
        R_cam2gripper,t_cam2gripper=cv2.calibrateHandEye(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,method=cv2.CALIB_HAND_EYE_DANIILIDIS)
        
        #print("R_cam2gripper: "+str(R_cam2gripper))
        #print("t_cam2gripper: "+str(t_cam2gripper))
        ecm_T_rightcam_cv=np.identity(4)
        ecm_T_rightcam_cv[0:3,0:3]=R_cam2gripper
        ecm_T_rightcam_cv[0:3,3]=t_cam2gripper.flatten()
        #print("ecm_T_rightcam_cv: "+str(ecm_T_rightcam_cv))
        ecm_T_rightcam_cv_list=ecm_T_rightcam_cv.tolist()

        ecm_T_rightcam_list=ecm_T_rightcam.tolist()
        data_right = {'ecm_T_rightcam': ecm_T_rightcam_list}
        #print("ecm_T_rightcam: "+str(ecm_T_rightcam_cv))

        #print("Frobenius norm of difference: "+str(np.linalg.norm(ecm_T_rightcam-ecm_T_rightcam_cv,'fro')))
        
        #angle_diff,translation_diff=decomposed_difference(ecm_T_rightcam,ecm_T_rightcam_cv)
        #print("Angle Difference: "+str(angle_diff))
        #print("Translation Difference: "+str(translation_diff))
        

        #Calculate accuracy of Right transformation 
        angle_diff_list=[]
        translation_diff_list=[]
        for i in range(len(rb_T_ecm_right)-1):
            A_raw=rightcam_T_scene[i]@utils.invHomogeneousNumpy(rightcam_T_scene[i+1])
            A_raw=utils.EnforceOrthogonalityNumpy_FullTransform(A_raw)
            #A_raw=np.dot(utils.invHomogeneousNumpy(rightcam_T_scene[i+1]),rightcam_T_scene[i]) 
            A_calc=utils.invHomogeneousNumpy(ecm_T_rightcam)@utils.invHomogeneousNumpy(rb_T_ecm_right[i])@rb_T_ecm_right[i+1]@ecm_T_rightcam
            A_calc=utils.EnforceOrthogonalityNumpy_FullTransform(A_calc)
            #A_calc=ecm_T_rightcam_cv@rb_T_ecm_right[i+1]@utils.invHomogeneousNumpy(rb_T_ecm_right[i])@utils.invHomogeneousNumpy(ecm_T_rightcam_cv)
            angle_diff,translation_diff=decomposed_difference(A_raw,A_calc)
            angle_diff=angle_diff*(180/np.pi)
            angle_diff_list.append(angle_diff)
            translation_diff_list.append(translation_diff)
        angle_diff_list=np.array(angle_diff_list,dtype='float32')
        translation_diff_list=np.array(translation_diff_list,dtype='float32')
        angle_diff=np.mean(angle_diff_list)
        translation_diff=np.mean(translation_diff_list)

        std_angle_diff=np.std(angle_diff_list)
        std_translation_diff=np.std(translation_diff_list)
        print("Angle Difference List Right: "+str(angle_diff_list))
        print("Translation Difference List Right: "+str(translation_diff_list))
        print("Mean Angle Difference Right: "+str(angle_diff))
        print("Mean Translation Difference Right: "+str(translation_diff))

        print("STD Angle Difference Right: "+str(std_angle_diff))
        print("STD Translation Difference Right: "+str(std_translation_diff))
        



        #Do Left Next
        A=[]
        B=[]
        R_gripper2base=[]
        t_gripper2base=[]
        R_target2cam=[]
        t_target2cam=[]
        #for i in range(len(rb_T_ecm_left)-1):
        for i in range(len(rb_T_ecm_left)-1):
            A_i=leftcam_T_scene[i]@utils.invHomogeneousNumpy(leftcam_T_scene[i+1]) #Best
            A_i=utils.EnforceOrthogonalityNumpy_FullTransform(A_i)
            #print("A_i: "+str(A_i))
            #B_i=np.dot(rb_T_ecm_right[i+1],utils.invHomogeneousNumpy(rb_T_ecm_right[i])) #Initial
            B_i=utils.invHomogeneousNumpy(rb_T_ecm_left[i])@rb_T_ecm_left[i+1] #Best
            B_i=utils.EnforceOrthogonalityNumpy_FullTransform(B_i)
            A.append(B_i)
            B.append(A_i)

            # R_gripper2base.append(rb_T_ecm_left[i][0:3,0:3])
            # t_gripper2base.append(rb_T_ecm_left[i][0:3,3])

            # R_target2cam.append(left_cam_T_scene_rvec_new[i])
            # t_target2cam.append(left_cam_T_scene_tvec_new[i])
            

            R_gripper2base.append(rb_T_ecm_left[i][0:3,0:3])
            t_gripper2base.append(rb_T_ecm_left[i][0:3,3])

            R_target2cam.append(leftcam_T_scene[i][0:3,0:3])
            t_target2cam.append(leftcam_T_scene[i][0:3,3])
        
        #Solve hand-eye problem
        A=np.array(A,dtype='float32')
        B=np.array(B,dtype='float32')
        R_gripper2base=np.array(R_gripper2base,dtype='float32')
        t_gripper2base=np.array(t_gripper2base,dtype='float32')
        R_target2cam=np.array(R_target2cam,dtype='float32')
        t_target2cam=np.array(t_target2cam,dtype='float32')

        ecm_T_leftcam=self.hand_eye.ComputeHandEye(A,B)
        #print("ecm_T_leftcam: "+str(ecm_T_leftcam))
        #data_left = {'ecm_T_leftcam': ecm_T_leftcam.tolist()}

        R_cam2gripper,t_cam2gripper=cv2.calibrateHandEye(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,method=cv2.CALIB_HAND_EYE_DANIILIDIS)
        #print("R_cam2gripper: "+str(R_cam2gripper))
        #print("t_cam2gripper: "+str(t_cam2gripper))
        ecm_T_leftcam_cv=np.identity(4)
        ecm_T_leftcam_cv[0:3,0:3]=R_cam2gripper
        ecm_T_leftcam_cv[0:3,3]=t_cam2gripper.flatten()
        #print("ecm_T_leftcam_cv: "+str(ecm_T_leftcam_cv))
        
        ecm_T_leftcam_cv_list=ecm_T_leftcam_cv.tolist()
        
        ecm_T_leftcam_list=ecm_T_leftcam.tolist()
        data_left = {'ecm_T_leftcam': ecm_T_leftcam_list}
        #print("ecm_T_leftcam: "+str(ecm_T_leftcam_cv))

        
        #Calculate accuracy of Left transformation 
        angle_diff_list=[]
        translation_diff_list=[]
        for i in range(len(rb_T_ecm_left)-1):
            A_raw=leftcam_T_scene[i]@utils.invHomogeneousNumpy(leftcam_T_scene[i+1])
            A_raw=utils.EnforceOrthogonalityNumpy_FullTransform(A_raw)
            #A_raw=np.dot(utils.invHomogeneousNumpy(rightcam_T_scene[i+1]),rightcam_T_scene[i]) 
            A_calc=utils.invHomogeneousNumpy(ecm_T_leftcam_cv)@utils.invHomogeneousNumpy(rb_T_ecm_left[i])@rb_T_ecm_left[i+1]@ecm_T_leftcam_cv
            A_calc=utils.EnforceOrthogonalityNumpy_FullTransform(A_calc)
            #A_calc=ecm_T_rightcam_cv@rb_T_ecm_right[i+1]@utils.invHomogeneousNumpy(rb_T_ecm_right[i])@utils.invHomogeneousNumpy(ecm_T_rightcam_cv)
            angle_diff,translation_diff=decomposed_difference(A_raw,A_calc)
            angle_diff=angle_diff*(180/np.pi)
            angle_diff_list.append(angle_diff)
            translation_diff_list.append(translation_diff)


        angle_diff_list=np.array(angle_diff_list,dtype='float32')
        translation_diff_list=np.array(translation_diff_list,dtype='float32')
        angle_diff=np.mean(angle_diff_list)
        translation_diff=np.mean(translation_diff_list)

        std_angle_diff=np.std(angle_diff_list)
        std_translation_diff=np.std(translation_diff_list)
        print("Angle Difference List Left: "+str(angle_diff_list))
        print("Translation Difference List Left: "+str(translation_diff_list))
        print("Angle Difference Left: "+str(angle_diff))
        print("Translation Difference Left: "+str(translation_diff))



        print("STD Angle Difference Left: "+str(std_angle_diff))
        print("STD Translation Difference Left: "+str(std_translation_diff))




        #Store these values
        with open(calibration_params_right+"hand_eye_calibration_right.yaml","w") as f:
                yaml.dump(data_right,f)

        with open(calibration_params_left+"hand_eye_calibration_left.yaml","w") as f:
                yaml.dump(data_left,f)  
        print("Finished Hand-Eye Calibration")

    def getFolderName(self):
        #Gets the most recent folder where we store checkerboards and base_T_ecm poses
        folder_count=1
        root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        old_root=root_directory
        if os.path.isdir(root_directory):
            while True:
                if os.path.isdir(root_directory):
                    old_root=root_directory
                    folder_count+=1
                    root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
                else:
                    break
            self.rootName=old_root
        else:
            self.calbration_message_label.config(text="No Available Checkerboards, grab some")
            return
    def createSaveFolder(self):
        #Creates Root Folder to Store New Checkerboard Images and base_T_ecm frames
        folder_count=1
        root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        while True:
            if os.path.isdir(root_directory):
                folder_count+=1
                root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)                    
            else:
                break
        os.mkdir(root_directory)
        self.rootName=root_directory

def decomposed_difference(A, B):
    # Extract rotation matrices and translation vectors
    RA, RB = A[:3, :3], B[:3, :3]
    tA, tB = A[:3, 3], B[:3, 3]
    translation_diff = np.linalg.norm(tA - tB)

    # Convert rotation matrices to quaternions
    quatA, quatB = Rotation.from_matrix(RA).as_quat(), Rotation.from_matrix(RB).as_quat()

    # Calculate the angular difference between quaternions
    #rotation_diff = Rotation.from_quat(quatA).inv() * Rotation.from_quat(quatB)
    #angle_diff = rotation_diff.magnitude()
    dot_product=np.abs(np.dot(quatA,quatB))
    dot_product=np.clip(dot_product,-1.0,1.0)
    angle_diff=2*np.arccos(dot_product)

    # Calculate the Euclidean distance between translation vectors
    

    return angle_diff, translation_diff


if __name__=='__main__':
    rospy.init_node('CameraCalibrator')
    rospy.Rate(10000)
   
    GUI=CameraCalibGUI()
    cv2.destroyAllWindows()

=======
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import tkinter as tk
import threading
import os
import yaml
from include import HandEye
from include import Renderer
from scipy.spatial.transform import Rotation

import dvrk
import tf_conversions.posemath as pm
import PyKDL
from include import utils
import cv2.aruco as aruco
#####To Do: Add a functionality to calibrate using an ArUco board, also add functionality to change size of checkerboard

ECM_FRAME_WIDTH_DESIRED=1024
ECM_FRAME_HEIGHT_DESIRED=722

##################Change These Variables If Needed#############################
#These Dimensions should be in meters, should change over to meters
ARUCO_IDs=[4,5,6,7] #List containing the IDs of the aruco markers that we are tracking
NUM_FRAME_DETECTIONS=8 #How many sets of aruco "frames" need to be detected, ToDo for later


CHECKERBOARD_DIM=(8,8) #Number of inner corners in the checkerboard (corners height, corners width)
CHECKER_WIDTH=0.0078232 #Width of each checker in the checkerboard (in meters)

RANSAC_SCENE_REPROJECTION_ERROR=0.0005 #Reprojection error for scene localization RANSAC (in meters)
RANSAC_SCENE_ITERATIONS=1000 #Number of iterations for scene localization RANSAC

#Rigid Body Definition of Ring Over Wire Aruco Holder, each four coordinates define an ArUco marker with corresponding ID:
#Marker corners are defined clockwise from top left
#3D model origin is the top left of AruCO ID 6
ARUCO_SIDELENGTH=0.0254508#0.0253492 #0.025527 #0.0253492 #in meters
ARUCO_SEPERATION=0.10226 #From closest edges, in meters
ARUCO_HEIGHT_OFFSET=0.005 ######!!!!!!!!!!Use Planar ArUco rig!!!!!!!!!!!!!!

RINGOWIRE_MODELPOINTS={
    "6":np.array([
    [0.0,0.0,0.0],
    [ARUCO_SIDELENGTH,0.0,0.0],
    [ARUCO_SIDELENGTH,-ARUCO_SIDELENGTH,0.0],
    [0.0,-ARUCO_SIDELENGTH,0.0]],dtype='float32'),

    "4":np.array([
    [0.0,ARUCO_SEPERATION+ARUCO_SIDELENGTH,ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH,ARUCO_SEPERATION+ARUCO_SIDELENGTH,ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH,ARUCO_SEPERATION,ARUCO_HEIGHT_OFFSET],
    [0.0,ARUCO_SEPERATION,ARUCO_HEIGHT_OFFSET]],dtype='float32'),

    "5":np.array([
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,2*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION+ARUCO_SIDELENGTH,2*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,2*ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,ARUCO_SEPERATION,2*ARUCO_HEIGHT_OFFSET]],dtype='float32'),

    "7":np.array([
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,3*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,0.0,3*ARUCO_HEIGHT_OFFSET],
    [2*ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SIDELENGTH,3*ARUCO_HEIGHT_OFFSET],
    [ARUCO_SIDELENGTH+ARUCO_SEPERATION,-ARUCO_SIDELENGTH,3*ARUCO_HEIGHT_OFFSET]],dtype='float32')
}


FRAMES_TO_REMOVE_RIGHT=[22,28]   #These frames were determined to be garbage visually
FRAMES_TO_REMOVE_LEFT=[22,26,28,31]
NUM_FRAMES_CAPTURED=41

REQUIRED_CHECKERBOARD_NUM=10 #Number of checkerboard images needed for the calibration
ERROR_THRESHOLD=10 #Pixel Error Threshold for centering ECM above checkerboard

#PROPORTIONAL_GAIN=20 #Gain for centering ECM above checkerboard
##MAX_ITERATIONS=40
#Where the images for the calibration are saved

CALIBRATION_DIR="../resources/Calib/" #Where we store calibration parameters and images


RIGHT_FRAMES_FILEDIR='/chessboard_images_right/'
LEFT_FRAMES_FILEDIR='/chessboard_images_left/'

#Where the calibration parameters are saved
RIGHT_CAMERA_CALIB_DIR="/calibration_params_right/"
LEFT_CAMERA_CALIB_DIR="/calibration_params_left/"

#Where we store the base_T_ecm poses
BASE_TO_ECM_DIR="/base_T_ecm_poses/"

#Ros Topics
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'


# Preset Motions from Starting Pose

'''
Sets of 7 motions for 3 different z axis increments:
Firt, increment z
then:
1. No rotation/Translation
2. Rotate about z by positive angle
3. Rotate about z by negative angle
4. Rotate about x by positive angle
5. Rotate about x by negative angle
6. Translate along x by positive val
7. translate along y by positive val
Repeat 3 times

We define these as numpy frames, they are later converted to PyKDL frames

'''

z_translation=0.0075 #Amount that we translate along z (1.5 centimeters)
Z_MOTION=[0,z_translation,z_translation*2,z_translation*3]
planar_translation=0.005 #Amount that we translate in plane parallel to endoscope
rotation_angle=2*(np.pi/180) #20 degrees in Rad

#T1, no change
T1=np.identity(4)
print("T1: "+str(T1))
#T2 rotation about z positive
Rz=utils.rotationZ(rotation_angle)
Rz=utils.EnforceOrthogonalityNumpy(Rz)
T2=np.identity(4)
T2[0:3,0:3]=Rz
print("T2: "+str(T2))
#T3 rotation about z negative
Rz=utils.rotationZ(-rotation_angle)
Rz=utils.EnforceOrthogonalityNumpy(Rz)
T3=np.identity(4)
T3[0:3,0:3]=Rz
print("T3: "+str(T3))

#T4 rotation about x positive
Rx=utils.rotationX(rotation_angle/2)
Rx=utils.EnforceOrthogonalityNumpy(Rx)
T4=np.identity(4)
T4[0:3,0:3]=Rx
print("T2: "+str(T4))
#T5 rotation about x negative
Rx=utils.rotationX(-rotation_angle/2)
Rx=utils.EnforceOrthogonalityNumpy(Rx)
T5=np.identity(4)
T5[0:3,0:3]=Rx
print("T3: "+str(T5))

#T4 translation along x
T6=np.identity(4)
T6[0,3]=planar_translation
print("T4: "+str(T6))
#T4 translation along y
T7=np.identity(4)
T7[1,3]=planar_translation
print("T5: "+str(T7))
MOTIONS=[T1,T2,T3,T4,T5,T6,T7]



class CameraCalibGUI:

    def __init__(self):

        self.bridge=CvBridge()

        self.rootName=None #Root folder that we are writing calibration parameters to

        #Counters
        self.ranOnce=0
        self.frame_number=0


        ######Aruco Things:
        self.aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_1000) #using the 4x4 dictionary to find markers
        self.aruco_params=aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshConstant=10



        #####################Setting Up GUI#####################
        self.window=tk.Tk()
        self.window.title("dVRK Camera Calibrator")
        self.window.rowconfigure([0,1,2,3,4],weight=1)
        self.window.columnconfigure([0,1],weight=1)

        #----Message Box
        self.message_label=tk.Label(text="Welcome to the dVRK Endoscope Calibrator",width=40)
        self.message_label.grid(row=0,column=0)
        self.message_label2=tk.Label(text="Performs Camera Calibration and Hand-Eye Calibration",width=60)
        self.message_label2.grid(row=1,column=0)

        #---Buttons

        #The grab frame callback automatically moves the robot to a new pose an grabs a frame
        self.button_frame=tk.Button(text="Grab Frame(s)",width=15,command=self.grabFramesCallback)
        self.button_frame.grid(row=2,column=0,sticky="nsew")

        self.button_savemanual=tk.Button(text="Save Manual",width=15,command=self.saveManualCallback)
        self.button_savemanual.grid(row=2,column=1,sticky="nsew")

        self.button_calibrate=tk.Button(text="Calibrate",width=15,command=self.calibrateCameraCallback)
        self.button_calibrate.grid(row=3,column=0,sticky="nsew")
        
        #Tick Box for Manual camera calibration
        self.checkbox_manual=tk.Checkbutton(text="Manual",onvalue=1,offvalue=0,command=self.manualCheckbox)
        self.checkbox_manual.grid(row=3,column=1,sticky='nsew')



        self.calbration_message_label=tk.Label(text="",width=60)
        self.calbration_message_label.grid(row=4,column=0)


        #Frames that we are reading from endoscope
        self.frameLeft=None
        self.frameRight=None
        self.isManual=False

        #Subscribing to ROS topics
        rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackRight,queue_size=1,buff_size=2**18)
        rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackLeft,queue_size=1,buff_size=2**18)
        
        #Setting up the ecm
        self.ecm=dvrk.ecm("ECM")
        enable_true=self.ecm.enable()
        home_true=self.ecm.home()
        print("Enable: "+str(enable_true))
        print("Home: "+str(home_true))



        #Checkerboard subpix criteria:
        self.criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 500, 0.001)


        


        #Execution Loop  
        #Window where we display the left/right frames:
        #cv2.namedWindow("Left/Right Frames",cv2.WINDOW_NORMAL)
        



        #Hand-Eye Parameteres
        self.hand_eye=HandEye.HandEye() #Creates hand-eye calibration object
        self.rb_T_ecm_list=[]   #List of robot base (cart) to ecm transforms
        
        self.mtx_right=None
        self.mtx_left=None
        self.dst_right=None
        self.dist_left=None
        self.rb_T_ecm_list=[]
        self.rb_T_ecm_setpoint_list=[]

        rospy.sleep(1)

        self.window.after(1,self.showFramesCallback)
        self.window.mainloop()
    
    def manualCheckbox(self):
        self.isManual=not self.isManual

    def showFramesCallback(self):

        if self.ranOnce>=2:
            #print("Entered")
            frameLeft=cv2.resize(self.frameLeft,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
            frameRight=cv2.resize(self.frameRight,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Left Frame",frameLeft)
            cv2.imshow("Right Frame",frameRight)
            # img_combined=cv2.hconcat([self.frameLeft,self.frameRight])
            # img_combined=cv2.resize(img_combined,(361,512),interpolation=cv2.INTER_LINEAR)
            # cv2.imshow("Left/Right Frames",img_combined)
            cv2.waitKey(1)
        self.window.after(1,self.showFramesCallback)
    
    #Callback for grabbing frames
    def frameCallbackRight(self,data):
        self.frameRight=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.frameRight=cv2.resize(self.frameRight,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
        self.ranOnce+=1

    def frameCallbackLeft(self,data):
        self.frameLeft=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.frameLeft=cv2.resize(self.frameLeft,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
        self.ranOnce+=1
        #print("New Frame")
    def findArUcos(self,frame):
        frame_gray=cv2.cvtColor(frame.copy(),cv2.COLOR_BGR2GRAY)
        corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)
        #corners=cv2.cornerSubPix(frame_gray,corners,(11,11),(-1,-1),self.criteria)
        corners_filtered=[]
        ids_filtered=[]
        if ids is not None:
            for id,corner in zip(ids,corners):
                if id[0] in ARUCO_IDs:
                    corners_filtered.append(corner)
                    ids_filtered.append([id[0]])
            ids=np.array(ids_filtered,dtype='float32')
            corners=np.array(corners_filtered,dtype='float32')
            if len(ids_filtered)>0:
                return ids,corners
            else:
                self.calbration_message_label.config(text="ArUco Out of View. Make sure to position ECM far enough above")
                return None,None
        else:
            self.calbration_message_label.config(text="ArUco Out of View. Make sure to position ECM far enough above")
            return None,None
    def findAxisErrors(self,frame,ids,corners):
        
        image_center=[frame.shape[1]/2,frame.shape[0]/2]
        corners_for_mean=None
        for corner in corners:
            if corners_for_mean is None:
                corners_for_mean=corner[0]
            else:
                corners_for_mean=np.vstack((corners_for_mean,corner[0]))
        #print("Corners for mean: "+str(corners_for_mean))
        checkerboard_center=np.mean(corners_for_mean,axis=0)
        #print("checkerboard_center: "+str(checkerboard_center))
        e_x=image_center[0]-checkerboard_center[0]
        e_y=image_center[1]-checkerboard_center[1]
        #Displaying the Checkerboard Center (red) and Camera Center (green)
        new_frame=frame
        new_frame=cv2.circle(new_frame,(int(checkerboard_center[0]),int(checkerboard_center[1])),3,(0,0,255),3)  #Drawing the checkerboard center
        new_frame=cv2.circle(new_frame,(int(image_center[0]),int(image_center[1])),ERROR_THRESHOLD,(0,255,0),2)  #Drawing the Camera Center
        new_frame=aruco.drawDetectedMarkers(frame,corners=corners,ids=ids)
        cv2.imshow('Alignment Error', new_frame)
        cv2.waitKey(50)
        return e_x,e_y

    def saveManualCallback(self):

        os.mkdir(self.rootName+BASE_TO_ECM_DIR)
        rb_T_ecm_store=np.array(self.rb_T_ecm_list,dtype='float32')
        print("rb_T_ecm_store: "+str(rb_T_ecm_store))
        np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm',rb_T_ecm_store)

        rb_T_ecm_setpoint_store=np.array(self.rb_T_ecm_setpoint_list,dtype='float32')
        np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm_setpoint',rb_T_ecm_setpoint_store)

        self.calbration_message_label.config(text="Frame Grabbing Done")

        self.rb_T_ecm_list=[]
        self.rb_T_ecm_setpoint_list=[]
        self.frame_number=0



    def grabFramesCallback(self):
        frameLeft=cv2.resize(self.frameLeft,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)
        frameRight=cv2.resize(self.frameRight,(ECM_FRAME_WIDTH_DESIRED,ECM_FRAME_HEIGHT_DESIRED),interpolation=cv2.INTER_LINEAR)

        if self.isManual:
            #Manually grab the frames
            if self.frame_number==0:
                #Initialize the save folder
                self.createSaveFolder()
                self.rb_T_ecm_list=[]
            
            file_name_right=self.rootName+RIGHT_FRAMES_FILEDIR
            file_name_left=self.rootName+LEFT_FRAMES_FILEDIR     
            if not os.path.isdir(file_name_right):
                os.mkdir(file_name_right)
                os.mkdir(file_name_left)
            
            ecm_pose=self.ecm.measured_cp()
            rb_T_ecm=pm.toMatrix(ecm_pose)
            self.rb_T_ecm_list.append(rb_T_ecm)

            ecm_pose=self.ecm.setpoint_cp()
            rb_T_ecm=pm.toMatrix(ecm_pose)
            self.rb_T_ecm_setpoint_list.append(rb_T_ecm)


            cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",frameRight)
            cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",frameLeft)

            self.frame_number+=1
            self.calbration_message_label.config(text="Frame Count: "+str(self.frame_number))
        else:


            #Automatically grab the frames and save them

            self.just_grabbed_frames=True
            self.createSaveFolder()    #This Creates a new folder with an incremented folder number (Calib_num)
            file_name_right=self.rootName+RIGHT_FRAMES_FILEDIR
            file_name_left=self.rootName+LEFT_FRAMES_FILEDIR

            #First we get user to align the ECM center with the checkerboard center
            self.calbration_message_label.config(text="Move ArUco Object Center (red) to Camera Center (green)")
            err_mag=ERROR_THRESHOLD+1
            print("Entered Grabbed Frames")
            while err_mag>ERROR_THRESHOLD: #Loops until center of LEFT ECM is within acceptable threshold
                ids,corners=self.findArUcos(self.frameLeft)
                if corners is not None:
                    e_x,e_y=self.findAxisErrors(self.frameLeft,ids,corners)
                    err_mag=np.sqrt((e_x**2)+(e_y**2))
                    print("err_mag: "+str(err_mag))

            print("Aligned")
            cv2.destroyWindow('Alignment Error')
            rospy.sleep(5)  #Sleep to allow user to move away from ecm

            #Now we capture the frames for the camera calibration
            if not os.path.isdir(file_name_right):
                os.mkdir(file_name_right)
                os.mkdir(file_name_left)
            print("File Name: "+str(file_name_right))
            self.frame_number=0
            #We loop through and move the ECM, then take frames, we also get the rb_T_ecm transform
            self.rb_T_ecm_list=[]
            translation_sign=-1
            
            ecm_pose_init=self.ecm.measured_cp()
            print("measured_cp")
            for i in range(len(Z_MOTION)): #Loop for 3 z values
                translation_sign=-1*translation_sign
                for j in range(len(MOTIONS)): #Loop for motions in this plane                
                    #Move it back to central location before translating again
                    print("starting move")
                    self.ecm.move_cp(ecm_pose_init).wait()
                    print("moving")
                    rospy.sleep(0.7)
                    ecm_pose_curr=self.ecm.measured_cp()
                    print("moved")

                    #print("ECM Position="+str(ecm_pose_curr.p))
                    #print("ECM Rotation="+str(ecm_pose_curr.M))
                    #print("z motion: "+str(Z_MOTION[i]))
                    #print("Tranform: "+str(MOTIONS[j]))
                    
                    motion_mat=pm.fromMatrix(MOTIONS[j])
                    
                    ecm_pose_curr.p[0]+=translation_sign*motion_mat.p[0]
                    ecm_pose_curr.p[1]+=translation_sign*motion_mat.p[1]
                    ecm_pose_curr.p[2]+=Z_MOTION[i] #increments z pose
                    
                    ecm_pose_curr.M=ecm_pose_curr.M*motion_mat.M
                    self.ecm.move_cp(ecm_pose_curr).wait()
                    rospy.sleep(1.5)                

                    #Grab the pose of ecm w.r.t. robot base
                    ecm_pose_new=self.ecm.measured_cp()
                    print("ecm_pose: "+str(ecm_pose_new))
                    rb_T_ecm=pm.toMatrix(ecm_pose_new)
                    print("ecm_pose numpy: "+str(rb_T_ecm))
                    self.rb_T_ecm_list.append(rb_T_ecm)

                    cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",self.frameRight)
                    cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",self.frameLeft)

                    self.frame_number+=1
            print("Num Frames: "+str(self.frame_number))
            #Store the rb_T_ecm poses in a yaml file
            os.mkdir(self.rootName+BASE_TO_ECM_DIR)
            rb_T_ecm_store=np.array(self.rb_T_ecm_list,dtype='float32')
            print("rb_T_ecm_store: "+str(rb_T_ecm_store))
            np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm',rb_T_ecm_store)

            rb_T_ecm_setpoint_store=np.array(self.rb_T_ecm_setpoint_list,dtype='float32')
            np.save(self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm_setpoint',rb_T_ecm_setpoint_store)

            self.calbration_message_label.config(text="Frame Grabbing Done")

                

 

        #First, servo ECM in 2D (along x,y plane) to have ECM center at checkerboard center (there is no z-axis movement)
        #For now only servo based on left camera
        #corners=self.findCheckerboard(self.frameLeft)
        #if corners is not None:
            #Get the rotation between 2x2 coordinate systems
        '''
        checkerboard_center_initial=np.mean(corners,axis=0) #Initial checkerboard center
        delta=np.array([0.04,0.04]) #How much we move ECM by (direction of how we expect it to move in camera coord system)
        ecm_pose_curr=self.ecm.setpoint_cp()
        ecm_pose_curr.p[0]+=delta[0]
        ecm_pose_curr.p[1]+=delta[1]
        self.ecm.move_cp(ecm_pose_curr).wait()
        rospy.sleep(0.05)
        corners=self.findCheckerboard(self.frameLeft)
        checkerboard_center_actual=np.mean(corners,axis=0)
        delta_actual=np.array([checkerboard_center_actual[0][0]-checkerboard_center_initial[0][0],checkerboard_center_actual[0][1]-checkerboard_center_initial[0][1]])

        t_hat=delta/np.sqrt((delta[0]**2)+(delta[1]**2))
        s_hat=delta_actual/np.sqrt((delta_actual[0]**2)+(delta_actual[1]**2))

        if np.linalg.norm(s_hat+t_hat)<10**(-6):
            alpha_mag=2*np.arctan(np.linalg.norm(s_hat-t_hat)/np.linalg.norm(s_hat+t_hat+10**(-4)))

        else:
            alpha_mag=2*np.arctan(np.linalg.norm(s_hat-t_hat)/np.linalg.norm(s_hat+t_hat))
        cross_prod=np.cross(t_hat,s_hat)
        print("Alpha Mag: "+str(alpha_mag*(180/np.pi)))
        alpha=np.sign(cross_prod)*alpha_mag
        theta_z=alpha
        print("Rotation Angle: "+str(theta_z*(180/np.pi)))
        Rz=np.array([
            [np.cos(theta_z),-np.sin(theta_z)],
            [np.sin(theta_z),np.cos(theta_z)]
        ])

        err_mag=ERROR_THRESHOLD+1
        '''
        '''
        #Finds mapping from change in camera coord system (pixels) to change in ECM coord system (meters) using regression
        move_ecm_list=[[0.0025,0.0025],[-0.005,0],[0,-0.005],[0.005,0.0025],[0,0.005],[0.0065,0],[-0.0055,-0.0045]]  #Set of 8 movements to write to ECM
        #move_ecm_list=[[0.01,0.01],[0.01,0],[0,0.01]]
        delta_ecm_list=[]
        delta_cam_list=[]
        corners=self.findCheckerboard(self.frameLeft)
        if corners is not None:
            checkerboard_center_initial=np.mean(corners,axis=0) #Initial checkerboard center
            ecm_pose_curr=self.ecm.measured_cp()
            for i in range(0,len(move_ecm_list)):  
                print("Move #: "+str(i))
                print("Move x: "+str(move_ecm_list[i][0])) 
                print("Move y: "+str(move_ecm_list[i][1]))               
                
                ecm_pose_curr.p[0]+=move_ecm_list[i][0]
                ecm_pose_curr.p[1]+=move_ecm_list[i][1]
                #self.ecm.move_cp(ecm_pose_curr)
                self.ecm.move_cp(ecm_pose_curr).wait()
                rospy.sleep(1)
                ecm_pose_new=self.ecm.measured_cp()

                delta_ecm_list.append([ecm_pose_new.p[0]-ecm_pose_curr.p[0],ecm_pose_new.p[1]-ecm_pose_curr.p[1]])
                print('Delta ECM List: '+str(delta_ecm_list))
                ecm_pose_curr=ecm_pose_new
            
                corners=self.findCheckerboard(self.frameLeft)
                if corners is not None:
                    checkerboard_center_new=np.mean(corners,axis=0)
                    delta_cam=[checkerboard_center_new[0][0]-checkerboard_center_initial[0][0],checkerboard_center_new[0][1]-checkerboard_center_initial[0][1]]
                    print("delta_cam: "+str(delta_cam))
                    delta_cam_list.append(delta_cam)
                    checkerboard_center_initial=checkerboard_center_new
                else:
                    print("Returned")
                    return  
            #Now we run the regression to get the transformation A such that: Ax=b 
            #where x=delta_cam and b=delta_ecm
            x_observations=np.array(delta_cam_list)
            b_observations=np.array(delta_ecm_list)
            A,residuals,rank,s=np.linalg.lstsq(x_observations,b_observations,rcond=None)
            print("Estimated A:"+str(A))
            print("Residual:"+str(residuals))

            
            err_mag=ERROR_THRESHOLD+1
            loop_count=0
            while err_mag>ERROR_THRESHOLD and loop_count<MAX_ITERATIONS: #Loops until center of LEFT ECM is within acceptable threshold
                corners=self.findCheckerboard(self.frameLeft)
                if corners is not None:
                    e_x,e_y=self.findAxisErrors(self.frameLeft,corners)
                    print("e_x,e_y: "+str([-e_x,-e_y]))

                    err_mag=np.sqrt((e_x**2)+(e_y**2))
                    error_vec=np.array([e_x,e_y]) #This is delta_cam
                    error_vec=PROPORTIONAL_GAIN*error_vec
                    print("error_vec: "+str(error_vec))
                    #move=PROPORTIONAL_GAIN*np.dot(Rz,error_vec)
                    delta_ecm=np.dot(A,error_vec) #A @ error_vec
                    print("move: "+str(delta_ecm))
                    ecm_pose_curr=self.ecm.measured_cp()
                    ecm_pose_curr.p[0]+=delta_ecm[0]
                    ecm_pose_curr.p[1]+=delta_ecm[1]
                    self.ecm.move_cp(ecm_pose_curr).wait()
                    #self.ecm.servo_cp(ecm_pose_curr)
                    rospy.sleep(0.05)
                    loop_count+=1


                else:
                    print("Returned")
                    return    

            print("Got to Middle")
            print("Grabbing Frames Starting")
            #Now we capture the frames for the camera calibration
            if not os.path.isdir(file_name_right):
                os.mkdir(file_name_right)
                os.mkdir(file_name_left)
            print("File Name: "+str(file_name_right))
            self.frame_number=1
            #We loop through and move the ECM, then take frames:
            for i in range(0,len(Z_MOTION)): #Loop for 3 z values
                for j in range(0,len(MOTIONS)): #Loop for motions in this plane
                    print("z motion: "+str(Z_MOTION[i]))
                    #print("motion: "+str(MOTIONS[j]))
                    ecm_pose_curr=self.ecm.measured_cp()
                    print("ECM Pose="+ecm_pose_curr.p)
                    ecm_pose_curr.p[2]+=Z_MOTION[i] #increments z pose

                    #ecm_pose_curr=ecm_pose_curr*pm.fromMatrix(MOTIONS[j])
                    self.ecm.move_cp(ecm_pose_curr).wait()
                    rospy.sleep(1)
                    cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",self.frameRight)
                    cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",self.frameLeft)
                    self.frame_number+=1
                    self.calibration_count_label.config(text="# of frames="+str(self.frame_number))


        else:
            print("Returned")
            return

        '''


        '''
        if not os.path.isdir(file_name_right):
            os.mkdir(file_name_right)
            os.mkdir(file_name_left)
        cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",self.frameRight)
        cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",self.frameLeft)
        self.frame_number+=1
        self.calibration_count_label.config(text="# of frames="+str(self.frame_number))
        '''
   
    def rightCheckbox(self):
        CameraCalibGUI.isRight=not CameraCalibGUI.isRight
        print("Callback")

    def leftCheckbox(self):
        CameraCalibGUI.isLeft=not CameraCalibGUI.isLeft


    def calibrateCameraCallback(self):
        #Does Both the Right and Left Cameras hand-eye+general calibration (using ArUco Markers)

        self.getFolderName()    #Gets Most Recent File Directory
        print("Root Name: "+self.rootName)
        frames_right_path=self.rootName+RIGHT_FRAMES_FILEDIR
        print("frames_right_path: "+frames_right_path)
        frames_left_path=self.rootName+LEFT_FRAMES_FILEDIR
        print("frames_left_path: "+frames_left_path)
        calibration_params_right=self.rootName+RIGHT_CAMERA_CALIB_DIR
        print("calibration_params_right: "+calibration_params_right)
        calibration_params_left=self.rootName+LEFT_CAMERA_CALIB_DIR
        print("calibration_params_left: "+calibration_params_left)
        frames_path=[frames_right_path,frames_left_path]
        params_path=[calibration_params_right,calibration_params_left]

        #################Camera Calibration###############
        #Repeats Twice, right first then left
        
        for i in range(2):
            calibration_params_path=params_path[i]
            checkerboard_frames_path=frames_path[i]

            if not os.path.isdir(calibration_params_path):
                os.mkdir(calibration_params_path)

            #Camera Calibration params
            objp = np.zeros((CHECKERBOARD_DIM[0]*CHECKERBOARD_DIM[1],3), np.float32)
            objp[:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)*CHECKER_WIDTH  #I think this is already the object point size as it should be

            objpoints=[]
            imgpoints=[]
            found=0
            for file in os.listdir(checkerboard_frames_path):
                filename=os.fsdecode(file)
                if filename.endswith(".jpg"):
                    path=checkerboard_frames_path+filename
                    img = cv2.imread(path) # Capture frame-by-frame
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    # Find the chess board corners
                    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_DIM, None)

                    # If found, add object points, image points (after refining them)
                    if ret == True:
                        objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
                        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
                        imgpoints.append(corners2)
                        # Draw and display the corners
                        img = cv2.drawChessboardCorners(img, CHECKERBOARD_DIM, corners2, ret)
                        found += 1
                        cv2.imshow('Chessboard Frame', img)
                        cv2.waitKey(5)
            
            if found>=REQUIRED_CHECKERBOARD_NUM:
                cv2.destroyWindow('Chessboard Frame')
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
                if i==0:
                    self.mtx_right=mtx
                    self.dst_right=dist
                if i==1:
                    self.mtx_left=mtx
                    self.dst_left=dist
            else:
                self.calbration_message_label.config(text="Note enough acceptable checkerboards, grab more frames, need "+str(REQUIRED_CHECKERBOARD_NUM))
                return
            
            #Finding Re-projection error
            error_list=[]
            for j in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[j], rvecs[j], tvecs[j], mtx, dist)
                error = cv2.norm(imgpoints[j], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                error_list.append(error)
            print("error list: "+str(error_list))
            error_list=np.array(error_list,dtype='float32')
            data = {'camera_matrix': np.asarray(mtx).tolist(),
                        'dist_coeff': np.asarray(dist).tolist(),
                        'mean reprojection error': float(np.mean(error_list)),
                        'std reprojection error': float(np.std(error_list))}
            
            #Writing data to file
            with open(calibration_params_path+"calibration_matrix.yaml","w") as f:
                yaml.dump(data,f)

        print("Finished Camera Calibration")




        ####################Hand Eye Calibration######################
        '''
        General Algorithm:
        Loop through and get each checkerboard, if it exists store pose of scene_T_camera and ecm_T_psm3 (do this for both left and right)
        Then convert these to the A (ecm poses) and B matrices (camera poses)
        '''
        rb_T_ecm_right=[]
        rb_T_ecm_left=[]

        rightcam_T_scene=[]
        leftcam_T_scene=[]

        right_cam_T_scene_rvec=[]
        right_cam_T_scene_tvec=[]

        left_cam_T_scene_rvec=[]
        left_cam_T_scene_tvec=[]

        #Open Up the rb_T_ecm list
        rb_T_ecm_path=self.rootName+BASE_TO_ECM_DIR+'rb_T_ecm_setpoint.npy'
        #print("rb_T_ecm_path: "+str(rb_T_ecm_path))
        rb_T_ecm_list=np.load(rb_T_ecm_path)

        #print("rb_T_ecm_list: "+str(rb_T_ecm_list))
        #Loop twice, right first then left
        for i in range(2):
            calibration_params_path=params_path[i]
            checkerboard_frames_path=frames_path[i]
            if i==0:
                #print("i="+str(i))
                mtx=self.mtx_right
                
                dist=self.dst_right
                frame_name='frame_right'
                #print("frame="+frame_name)
            elif i==1:
                #print("i="+str(i))
                frame_name='frame_left'
                mtx=self.mtx_left
                dist=self.dst_left
                #print("frame="+frame_name)
            
            #Loops for each ArUco that we previously captured
            for frame_num in range(NUM_FRAMES_CAPTURED):
                #print('frame_num: '+str(frame_num))
                filename=checkerboard_frames_path+frame_name+str(frame_num)+".jpg"
                img = cv2.imread(filename) # Capture frame-by-frame
                frame_gray=cv2.cvtColor(img.copy(),cv2.COLOR_BGR2GRAY)
                corners,ids,rejected=aruco.detectMarkers(frame_gray,dictionary=self.aruco_dict,parameters=self.aruco_params)
                


                if ids is not None: #We found IDs
                    '''
                    corners=np.array(list(corners),dtype='float32')
                    corners=corners.reshape(-1,4,2)
                    dim1=corners.shape[0]
                    dim2=corners.shape[1]
                    corners=corners.reshape(dim1*dim2,2)
                    corners=cv2.cornerSubPix(frame_gray,corners,(5,5),(-1,-1),self.criteria)                    
                    corners=corners.reshape(dim1,dim2,2)                    
                    corners=corners.reshape(dim1,1,dim2,2)                    
                    corners=tuple(corners)
                    '''


                    #Keeps only the ids that we want
                    corners_filtered=[]
                    ids_filtered=[]
                    for id,corner in zip(ids,corners):
                        if id[0] in ARUCO_IDs:
                            corners_filtered.append(corner)
                            ids_filtered.append([id[0]])


                    image_points=None
                    model_points=None
                    if len(ids_filtered)>0: #We found IDs after filtering
                        ids_print=np.array(ids_filtered)
                        corners_print=np.array(corners_filtered,dtype='float32')
                        #Display the corners:
                        frame_converted=aruco.drawDetectedMarkers(img,corners=corners_print,ids=ids_print)
                        #cv2.imshow('ArUco Frame', frame_converted)
                        #cv2.waitKey(500)
                        #Now we append the corresponding model and object points to these two arrays:
                        for id,corner in zip(ids_filtered,corners_filtered):
                            if image_points is None:
                                image_points=corner[0]
                                model_points=RINGOWIRE_MODELPOINTS[str(id[0])]
                            else:
                                image_points=np.vstack((image_points,corner[0]))
                                model_points=np.vstack((model_points,RINGOWIRE_MODELPOINTS[str(id[0])]))
                        #print("image_points"+str(image_points))
                        #print("model_points"+str(model_points))
                        #print("ids_filtered"+str(ids_filtered))

                        success,rotation_vector,translation_vector,_=cv2.solvePnPRansac(model_points,image_points,mtx,dist,\
                                                                                        iterationsCount=RANSAC_SCENE_ITERATIONS,reprojectionError=RANSAC_SCENE_REPROJECTION_ERROR,flags=cv2.USAC_MAGSAC)
                        

                        #Convert rotation_vector and translation_vector to homogeneous transform
                        if success:
                            frame_converted=img.copy()
                            frame_converted=cv2.drawFrameAxes(frame_converted,mtx,\
                                      dist,rotation_vector,translation_vector,0.05)
                            cv2.imshow('Pose',frame_converted)
                            cv2.waitKey(1)
                            #print("rotation_vector: "+str(rotation_vector))
                            #print("translation_vector: "+str(translation_vector))
                            cam_T_scene=utils.convertRvecTvectoHomo(rotation_vector,translation_vector)
                            #cam_T_scene=utils.invHomogeneousNumpy(cam_T_scene)
                            #cam_T_scene=utils.invHomogeneousNumpy(cam_T_scene)
                            #cam_T_scene=utils.EnforceOrthogonalityNumpy_FullTransform(cam_T_scene)

                            rb_T_ecm=rb_T_ecm_list[frame_num]
                            
                            rb_T_ecm=utils.EnforceOrthogonalityNumpy_FullTransform(rb_T_ecm)
                            #print('rb_T_ecm: '+str(rb_T_ecm))
                            #rb_T_ecm=utils.EnforceOrthogonalityNumpy_FullTransform(rb_T_ecm)

                            #Updating the lists
                            if i==0: #right
                                rb_T_ecm_right.append(rb_T_ecm)
                                rightcam_T_scene.append(cam_T_scene)
                                right_cam_T_scene_rvec.append(rotation_vector)
                                right_cam_T_scene_tvec.append(translation_vector)
                            elif i==1:
                                rb_T_ecm_left.append(rb_T_ecm)
                                leftcam_T_scene.append(cam_T_scene)
                                left_cam_T_scene_rvec.append(rotation_vector)
                                left_cam_T_scene_tvec.append(translation_vector)


        #Filter out bad frames in left and right
        rb_T_ecm_right_new=[]
        rightcam_T_scene_new=[]
        right_cam_T_scene_rvec_new=[]
        right_cam_T_scene_tvec_new=[]

        for i in range(len(rb_T_ecm_right)):
            if i not in FRAMES_TO_REMOVE_RIGHT:
                #print("i: "+str(i))
                rb_T_ecm_right_new.append(rb_T_ecm_right[i])
                rightcam_T_scene_new.append(rightcam_T_scene[i])
                right_cam_T_scene_rvec_new.append(right_cam_T_scene_rvec[i])
                right_cam_T_scene_tvec_new.append(right_cam_T_scene_tvec[i])

        rb_T_ecm_right=rb_T_ecm_right_new
        rightcam_T_scene=rightcam_T_scene_new

        left_cam_T_scene_rvec_new=[]
        left_cam_T_scene_tvec_new=[]
        rb_T_ecm_left_new=[]
        leftcam_T_scene_new=[]
        for i in range(len(rb_T_ecm_left)):
            if i not in FRAMES_TO_REMOVE_LEFT:
                rb_T_ecm_left_new.append(rb_T_ecm_left[i])
                leftcam_T_scene_new.append(leftcam_T_scene[i])
                left_cam_T_scene_rvec_new.append(left_cam_T_scene_rvec[i])
                left_cam_T_scene_tvec_new.append(left_cam_T_scene_tvec[i])
        rb_T_ecm_left=rb_T_ecm_left_new
        leftcam_T_scene=leftcam_T_scene_new



        #Solving Hand-Eye Calibration for both the right and left cameras (for now just use the OpenCV method)
        
        #Do Right First
        A=[]
        B=[]
        cv2.destroyWindow('Pose')
        R_gripper2base=[]
        t_gripper2base=[]
        R_target2cam=[]
        t_target2cam=[]


        #for i in range(len(rb_T_ecm_right)-1):
        for i in range(len(rb_T_ecm_right)-1):

            #A_i=np.dot(utils.invHomogeneousNumpy(rightcam_T_scene[i+1]),rightcam_T_scene[i]) #Initial
            A_i=rightcam_T_scene[i]@utils.invHomogeneousNumpy(rightcam_T_scene[i+1]) #Best
            A_i=utils.EnforceOrthogonalityNumpy_FullTransform(A_i)
            #print("A_i: "+str(A_i))
            #B_i=np.dot(rb_T_ecm_right[i+1],utils.invHomogeneousNumpy(rb_T_ecm_right[i])) #Initial

            B_i=utils.invHomogeneousNumpy(rb_T_ecm_right[i])@rb_T_ecm_right[i+1] #Best
            B_i=utils.EnforceOrthogonalityNumpy_FullTransform(B_i)

            A.append(B_i)
            B.append(A_i)
            #print("B_i: "+str(B_i))

            # R_gripper2base.append(rb_T_ecm_right[i][0:3,0:3])
            # t_gripper2base.append(rb_T_ecm_right[i][0:3,3])
            
            # R_target2cam.append(right_cam_T_scene_rvec_new[i])
            # t_target2cam.append(right_cam_T_scene_tvec_new[i])

            R_gripper2base.append(rb_T_ecm_right[i][0:3,0:3])
            t_gripper2base.append(rb_T_ecm_right[i][0:3,3])

            R_target2cam.append(rightcam_T_scene[i][0:3,0:3])
            t_target2cam.append(rightcam_T_scene[i][0:3,3])


        
        #Solve hand-eye problem
        A=np.array(A,dtype='float32')
        B=np.array(B,dtype='float32')
        R_gripper2base=np.array(R_gripper2base,dtype='float32')
        t_gripper2base=np.array(t_gripper2base,dtype='float32')
        R_target2cam=np.array(R_target2cam,dtype='float32')
        t_target2cam=np.array(t_target2cam,dtype='float32')
        
        ecm_T_rightcam=self.hand_eye.ComputeHandEye(A,B)
        #print("ecm_T_rightcam: "+str(ecm_T_rightcam))
        #data_right = {'ecm_T_rightcam': ecm_T_rightcam.tolist()}

        #print("R_gripper2base: "+str(R_gripper2base))
        #print("t_gripper2base: "+str(t_gripper2base))
        R_cam2gripper,t_cam2gripper=cv2.calibrateHandEye(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,method=cv2.CALIB_HAND_EYE_DANIILIDIS)
        
        #print("R_cam2gripper: "+str(R_cam2gripper))
        #print("t_cam2gripper: "+str(t_cam2gripper))
        ecm_T_rightcam_cv=np.identity(4)
        ecm_T_rightcam_cv[0:3,0:3]=R_cam2gripper
        ecm_T_rightcam_cv[0:3,3]=t_cam2gripper.flatten()
        #print("ecm_T_rightcam_cv: "+str(ecm_T_rightcam_cv))
        ecm_T_rightcam_cv_list=ecm_T_rightcam_cv.tolist()

        ecm_T_rightcam_list=ecm_T_rightcam.tolist()
        data_right = {'ecm_T_rightcam': ecm_T_rightcam_list}
        #print("ecm_T_rightcam: "+str(ecm_T_rightcam_cv))

        #print("Frobenius norm of difference: "+str(np.linalg.norm(ecm_T_rightcam-ecm_T_rightcam_cv,'fro')))
        
        #angle_diff,translation_diff=decomposed_difference(ecm_T_rightcam,ecm_T_rightcam_cv)
        #print("Angle Difference: "+str(angle_diff))
        #print("Translation Difference: "+str(translation_diff))
        

        #Calculate accuracy of Right transformation 
        angle_diff_list=[]
        translation_diff_list=[]
        for i in range(len(rb_T_ecm_right)-1):
            A_raw=rightcam_T_scene[i]@utils.invHomogeneousNumpy(rightcam_T_scene[i+1])
            A_raw=utils.EnforceOrthogonalityNumpy_FullTransform(A_raw)
            #A_raw=np.dot(utils.invHomogeneousNumpy(rightcam_T_scene[i+1]),rightcam_T_scene[i]) 
            A_calc=utils.invHomogeneousNumpy(ecm_T_rightcam)@utils.invHomogeneousNumpy(rb_T_ecm_right[i])@rb_T_ecm_right[i+1]@ecm_T_rightcam
            A_calc=utils.EnforceOrthogonalityNumpy_FullTransform(A_calc)
            #A_calc=ecm_T_rightcam_cv@rb_T_ecm_right[i+1]@utils.invHomogeneousNumpy(rb_T_ecm_right[i])@utils.invHomogeneousNumpy(ecm_T_rightcam_cv)
            angle_diff,translation_diff=decomposed_difference(A_raw,A_calc)
            angle_diff=angle_diff*(180/np.pi)
            angle_diff_list.append(angle_diff)
            translation_diff_list.append(translation_diff)
        angle_diff_list=np.array(angle_diff_list,dtype='float32')
        translation_diff_list=np.array(translation_diff_list,dtype='float32')
        angle_diff=np.mean(angle_diff_list)
        translation_diff=np.mean(translation_diff_list)

        std_angle_diff=np.std(angle_diff_list)
        std_translation_diff=np.std(translation_diff_list)
        print("Angle Difference List Right: "+str(angle_diff_list))
        print("Translation Difference List Right: "+str(translation_diff_list))
        print("Mean Angle Difference Right: "+str(angle_diff))
        print("Mean Translation Difference Right: "+str(translation_diff))

        print("STD Angle Difference Right: "+str(std_angle_diff))
        print("STD Translation Difference Right: "+str(std_translation_diff))
        



        #Do Left Next
        A=[]
        B=[]
        R_gripper2base=[]
        t_gripper2base=[]
        R_target2cam=[]
        t_target2cam=[]
        #for i in range(len(rb_T_ecm_left)-1):
        for i in range(len(rb_T_ecm_left)-1):
            A_i=leftcam_T_scene[i]@utils.invHomogeneousNumpy(leftcam_T_scene[i+1]) #Best
            A_i=utils.EnforceOrthogonalityNumpy_FullTransform(A_i)
            #print("A_i: "+str(A_i))
            #B_i=np.dot(rb_T_ecm_right[i+1],utils.invHomogeneousNumpy(rb_T_ecm_right[i])) #Initial
            B_i=utils.invHomogeneousNumpy(rb_T_ecm_left[i])@rb_T_ecm_left[i+1] #Best
            B_i=utils.EnforceOrthogonalityNumpy_FullTransform(B_i)
            A.append(B_i)
            B.append(A_i)

            # R_gripper2base.append(rb_T_ecm_left[i][0:3,0:3])
            # t_gripper2base.append(rb_T_ecm_left[i][0:3,3])

            # R_target2cam.append(left_cam_T_scene_rvec_new[i])
            # t_target2cam.append(left_cam_T_scene_tvec_new[i])
            

            R_gripper2base.append(rb_T_ecm_left[i][0:3,0:3])
            t_gripper2base.append(rb_T_ecm_left[i][0:3,3])

            R_target2cam.append(leftcam_T_scene[i][0:3,0:3])
            t_target2cam.append(leftcam_T_scene[i][0:3,3])
        
        #Solve hand-eye problem
        A=np.array(A,dtype='float32')
        B=np.array(B,dtype='float32')
        R_gripper2base=np.array(R_gripper2base,dtype='float32')
        t_gripper2base=np.array(t_gripper2base,dtype='float32')
        R_target2cam=np.array(R_target2cam,dtype='float32')
        t_target2cam=np.array(t_target2cam,dtype='float32')

        ecm_T_leftcam=self.hand_eye.ComputeHandEye(A,B)
        #print("ecm_T_leftcam: "+str(ecm_T_leftcam))
        #data_left = {'ecm_T_leftcam': ecm_T_leftcam.tolist()}

        R_cam2gripper,t_cam2gripper=cv2.calibrateHandEye(R_gripper2base,t_gripper2base,R_target2cam,t_target2cam,method=cv2.CALIB_HAND_EYE_DANIILIDIS)
        #print("R_cam2gripper: "+str(R_cam2gripper))
        #print("t_cam2gripper: "+str(t_cam2gripper))
        ecm_T_leftcam_cv=np.identity(4)
        ecm_T_leftcam_cv[0:3,0:3]=R_cam2gripper
        ecm_T_leftcam_cv[0:3,3]=t_cam2gripper.flatten()
        #print("ecm_T_leftcam_cv: "+str(ecm_T_leftcam_cv))
        
        ecm_T_leftcam_cv_list=ecm_T_leftcam_cv.tolist()
        
        ecm_T_leftcam_list=ecm_T_leftcam.tolist()
        data_left = {'ecm_T_leftcam': ecm_T_leftcam_list}
        #print("ecm_T_leftcam: "+str(ecm_T_leftcam_cv))

        
        #Calculate accuracy of Left transformation 
        angle_diff_list=[]
        translation_diff_list=[]
        for i in range(len(rb_T_ecm_left)-1):
            A_raw=leftcam_T_scene[i]@utils.invHomogeneousNumpy(leftcam_T_scene[i+1])
            A_raw=utils.EnforceOrthogonalityNumpy_FullTransform(A_raw)
            #A_raw=np.dot(utils.invHomogeneousNumpy(rightcam_T_scene[i+1]),rightcam_T_scene[i]) 
            A_calc=utils.invHomogeneousNumpy(ecm_T_leftcam_cv)@utils.invHomogeneousNumpy(rb_T_ecm_left[i])@rb_T_ecm_left[i+1]@ecm_T_leftcam_cv
            A_calc=utils.EnforceOrthogonalityNumpy_FullTransform(A_calc)
            #A_calc=ecm_T_rightcam_cv@rb_T_ecm_right[i+1]@utils.invHomogeneousNumpy(rb_T_ecm_right[i])@utils.invHomogeneousNumpy(ecm_T_rightcam_cv)
            angle_diff,translation_diff=decomposed_difference(A_raw,A_calc)
            angle_diff=angle_diff*(180/np.pi)
            angle_diff_list.append(angle_diff)
            translation_diff_list.append(translation_diff)


        angle_diff_list=np.array(angle_diff_list,dtype='float32')
        translation_diff_list=np.array(translation_diff_list,dtype='float32')
        angle_diff=np.mean(angle_diff_list)
        translation_diff=np.mean(translation_diff_list)

        std_angle_diff=np.std(angle_diff_list)
        std_translation_diff=np.std(translation_diff_list)
        print("Angle Difference List Left: "+str(angle_diff_list))
        print("Translation Difference List Left: "+str(translation_diff_list))
        print("Angle Difference Left: "+str(angle_diff))
        print("Translation Difference Left: "+str(translation_diff))



        print("STD Angle Difference Left: "+str(std_angle_diff))
        print("STD Translation Difference Left: "+str(std_translation_diff))




        #Store these values
        with open(calibration_params_right+"hand_eye_calibration_right.yaml","w") as f:
                yaml.dump(data_right,f)

        with open(calibration_params_left+"hand_eye_calibration_left.yaml","w") as f:
                yaml.dump(data_left,f)  
        print("Finished Hand-Eye Calibration")

    def getFolderName(self):
        #Gets the most recent folder where we store checkerboards and base_T_ecm poses
        folder_count=1
        root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        old_root=root_directory
        if os.path.isdir(root_directory):
            while True:
                if os.path.isdir(root_directory):
                    old_root=root_directory
                    folder_count+=1
                    root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
                else:
                    break
            self.rootName=old_root
        else:
            self.calbration_message_label.config(text="No Available Checkerboards, grab some")
            return
    def createSaveFolder(self):
        #Creates Root Folder to Store New Checkerboard Images and base_T_ecm frames
        folder_count=1
        root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)
        while True:
            if os.path.isdir(root_directory):
                folder_count+=1
                root_directory=CALIBRATION_DIR+'Calib_'+str(folder_count)                    
            else:
                break
        os.mkdir(root_directory)
        self.rootName=root_directory

def decomposed_difference(A, B):
    # Extract rotation matrices and translation vectors
    RA, RB = A[:3, :3], B[:3, :3]
    tA, tB = A[:3, 3], B[:3, 3]
    translation_diff = np.linalg.norm(tA - tB)

    # Convert rotation matrices to quaternions
    quatA, quatB = Rotation.from_matrix(RA).as_quat(), Rotation.from_matrix(RB).as_quat()

    # Calculate the angular difference between quaternions
    #rotation_diff = Rotation.from_quat(quatA).inv() * Rotation.from_quat(quatB)
    #angle_diff = rotation_diff.magnitude()
    dot_product=np.abs(np.dot(quatA,quatB))
    dot_product=np.clip(dot_product,-1.0,1.0)
    angle_diff=2*np.arccos(dot_product)

    # Calculate the Euclidean distance between translation vectors
    

    return angle_diff, translation_diff


if __name__=='__main__':
    rospy.init_node('CameraCalibrator')
    rospy.Rate(10000)
   
    GUI=CameraCalibGUI()
    cv2.destroyAllWindows()

>>>>>>> cb2dc77b51fe54e55eb17648b0794c2400e6f547
