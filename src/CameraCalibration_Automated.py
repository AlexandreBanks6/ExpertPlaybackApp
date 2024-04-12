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

import dvrk
import tf_conversions.posemath as pm
import PyKDL
from include import utils

#####To Do: Add a functionality to calibrate using an ArUco board, also add functionality to change size of checkerboard


##################Change These Variables If Needed#############################
#These Dimensions should be in meters, should change over to meters
CHECKERBOARD_DIM=(8,8) #Number of inner corners in the checkerboard (corners height, corners width)
CHECKER_WIDTH=0.0078232 #Width of each checker in the checkerboard


RANSAC_SCENE_REPROJECTION_ERROR=0.005 #Reprojection error for scene localization RANSAC (in meters)
RANSAC_SCENE_ITERATIONS=40 #Number of iterations for scene localization RANSAC

REQUIRED_CHECKERBOARD_NUM=10 #Number of checkerboard images needed for the calibration
ERROR_THRESHOLD=10 #Pixel Error Threshold for centering ECM above checkerboard

#PROPORTIONAL_GAIN=20 #Gain for centering ECM above checkerboard
##MAX_ITERATIONS=40
#Where the images for the calibration are saved

CALIBRATION_DIR="../resources/Calib/" #Where we store calibration parameters and images


RIGHT_FRAMES_FILEDIR="/chessboard_images_right/"
LEFT_FRAMES_FILEDIR="/chessboard_images_left/"

#Where the calibration parameters are saved
RIGHT_CAMERA_CALIB_DIR="/calibration_params_right/"
LEFT_CAMERA_CALIB_DIR="/calibration_params_left/"

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

z_translation=0.005 #Amount that we translate along z (2.5 centimeters)
Z_MOTION=[0,z_translation,z_translation*2]
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
Rx=utils.rotationX(rotation_angle)
Rx=utils.EnforceOrthogonalityNumpy(Rx)
T4=np.identity(4)
T4[0:3,0:3]=Rx
print("T2: "+str(T4))
#T5 rotation about x negative
Rx=utils.rotationX(-rotation_angle)
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
        self.button_frame=tk.Button(text="Grab Frames",width=15,command=self.grabFramesCallback)
        self.button_frame.grid(row=2,column=0,sticky="nsew")

        self.button_calibrate=tk.Button(text="Calibrate",width=15,command=self.calibrateCameraCallback)
        self.button_calibrate.grid(row=3,column=0,sticky="nsew")
        
        self.calibration_count_label=tk.Label(text="",width=60)
        self.calibration_count_label.grid(row=3,column=1)

        self.calbration_message_label=tk.Label(text="",width=60)
        self.calbration_message_label.grid(row=4,column=0)

        #Frames that we are reading from endoscope
        self.frameLeft=None
        self.frameRight=None

        #Subscribing to ROS topics
        rospy.Subscriber(name = RightFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackRight,queue_size=1,buff_size=2**18)
        rospy.Subscriber(name = LeftFrame_Topic, data_class=CompressedImage, callback=self.frameCallbackLeft,queue_size=1,buff_size=2**18)
        
        #Setting up the ecm
        self.ecm=dvrk.ecm("ECM")
        enable_true=self.ecm.enable()
        home_true=self.ecm.home()
        print("Enable: "+str(enable_true))
        print("Home: "+str(home_true))



        #Setting up PSM3
        self.psm3=dvrk.psm("PSM3")
        self.psm3.enable()
        self.psm3.home()


        #Checkerboard subpix criteria:
        self.criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


        #Execution Loop  
        #Window where we display the left/right frames:
        cv2.namedWindow("Left/Right Frames",cv2.WINDOW_NORMAL)
        
        self.window.after(1,self.showFramesCallback)
        self.window.mainloop()


        #Hand-Eye Parameteres
        self.hand_eye=HandEye.HandEye() #Creates hand-eye calibration object
        self.ecm_T_psm1_list=[]
        
        self.mtx_right=None
        self.mtx_left=None
        self.dst_right=None
        self.dist_left=None
        rospy.sleep(1)
    
    def showFramesCallback(self):

        if self.ranOnce>=2:
            img_combined=cv2.hconcat([self.frameLeft,self.frameRight])
            img_combined=cv2.resize(img_combined,(700,246),interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Left/Right Frames",img_combined)
            cv2.waitKey(1)
        self.window.after(1,self.showFramesCallback)
    
    #Callback for grabbing frames
    def frameCallbackRight(self,data):
        self.frameRight=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.ranOnce+=1

    def frameCallbackLeft(self,data):
        self.frameLeft=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.ranOnce+=1
        print("New Frame")
    def findCheckerboard(self,frame):
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,corners=cv2.findChessboardCorners(gray,CHECKERBOARD_DIM,None)
        if ret:
            corners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
            return corners
        else:
            self.calbration_message_label.config(text="Checkerboard Out of View. Make sure to position ECM far enough above")
            return None
    def findAxisErrors(self,frame,corners):
        image_center=[frame.shape[1]/2,frame.shape[0]/2]
        checkerboard_center=np.mean(corners,axis=0)
        e_x=image_center[0]-checkerboard_center[0][0]
        e_y=image_center[1]-checkerboard_center[0][1]
        #Displaying the Checkerboard Center (red) and Camera Center (green)
        new_frame=frame
        new_frame=cv2.circle(new_frame,(int(checkerboard_center[0][0]),int(checkerboard_center[0][1])),3,(0,0,255),3)  #Drawing the checkerboard center
        new_frame=cv2.circle(new_frame,(int(image_center[0]),int(image_center[1])),ERROR_THRESHOLD,(0,255,0),2)  #Drawing the Camera Center
        cv2.imshow('Alignment Error', new_frame)
        cv2.waitKey(100)
        return e_x,e_y

    def grabFramesCallback(self):
        self.getFolderName()
        file_name_right=self.rootName+RIGHT_FRAMES_FILEDIR
        file_name_left=self.rootName+LEFT_FRAMES_FILEDIR

        #First we get user to align the ECM center with the checkerboard center
        self.calbration_message_label.config(text="Move Checkerboard Center (red) to Camera Center (green)")
        err_mag=ERROR_THRESHOLD+1

        while err_mag>ERROR_THRESHOLD: #Loops until center of LEFT ECM is within acceptable threshold
            corners=self.findCheckerboard(self.frameLeft)
            if corners is not None:
                e_x,e_y=self.findAxisErrors(self.frameLeft,corners)
                err_mag=np.sqrt((e_x**2)+(e_y**2))
                print("err_mag: "+str(err_mag))
            else:
                print("Returned")
                return  

        print("Aligned")
        cv2.destroyWindow('Alignment Error')
        #Now we capture the frames for the camera calibration
        if not os.path.isdir(file_name_right):
            os.mkdir(file_name_right)
            os.mkdir(file_name_left)
        print("File Name: "+str(file_name_right))
        self.frame_number=0
        #We loop through and move the ECM, then take frames, we also get the ecm_T_psm3 transform
        self.ecm_T_psm1_list=[]
        translation_sign=1
        for i in range(len(Z_MOTION)): #Loop for 3 z values
            for j in range(len(MOTIONS)): #Loop for motions in this plane
                
                
                ecm_pose_curr=self.ecm.measured_cp()
                print("ECM Position="+str(ecm_pose_curr.p))
                print("ECM Rotation="+str(ecm_pose_curr.M))

                print("z motion: "+str(Z_MOTION[i]))
                print("Tranform: "+str(MOTIONS[j]))
                ecm_pose_curr.p[2]+=Z_MOTION[i] #increments z pose
                motion_mat=pm.fromMatrix(MOTIONS[j])
                ecm_pose_curr.p[0]+=translation_sign*motion_mat.p[0]
                ecm_pose_curr.p[1]+=translation_sign*motion_mat.p[1]
                translation_sign=-1*translation_sign
                ecm_pose_curr.M=ecm_pose_curr.M*motion_mat.M
                self.ecm.move_cp(ecm_pose_curr).wait()
                rospy.sleep(1)                
                cv2.imwrite(file_name_right+"frame_right"+str(self.frame_number)+".jpg",self.frameRight)
                cv2.imwrite(file_name_left+"frame_left"+str(self.frame_number)+".jpg",self.frameLeft)

                self.frame_number+=1
                self.calibration_count_label.config(text="# of frames="+str(self.frame_number))
                #Grab the pose of ecm_T_psm3
                ecm_T_psm3=self.psm3.setpoint_cp()      #This might have to be the ecm pose, should look at math if changed to this
                ecm_T_psm3=pm.fromMatrix(ecm_T_psm3)
                self.ecm_T_psm1_list.append(ecm_T_psm3)

                

 

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
        #Does Both the Right and Left Cameras hand-eye+general calibration

        self.getFolderName()
        frames_right_path=self.rootName+RIGHT_FRAMES_FILEDIR
        frames_left_path=self.rootName+LEFT_FRAMES_FILEDIR

        calibration_params_right=self.rootName+RIGHT_CAMERA_CALIB_DIR
        calibration_params_left=self.rootName+LEFT_CAMERA_CALIB_DIR

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
                        cv2.waitKey(500)
            
            if found>=REQUIRED_CHECKERBOARD_NUM:
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
            mean_error = 0
            for j in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[j], rvecs[j], tvecs[j], mtx, dist)
                error = cv2.norm(imgpoints[j], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error
            data = {'camera_matrix': np.asarray(mtx).tolist(),
                        'dist_coeff': np.asarray(dist).tolist(),
                        'mea reprojection error': [mean_error/len(objpoints)]}
            
            #Writing data to file
            with open(calibration_params_path+"calibration_matrix.yaml","w") as f:
                yaml.dump(data,f)

        ####################Hand Eye Calibration######################
        '''
        General Algorithm:
        Loop through and get each checkerboard, if it exists store pose of scene_T_camera and ecm_T_psm3 (do this for both left and right)
        Then convert these to the A (ecm poses) and B matrices (camera poses)
        '''
        ecm_T_psm3_right=[]
        ecm_T_psm3_left=[]

        scene_T_rightcam=[]
        scene_T_leftcam=[]

        objp = np.zeros((CHECKERBOARD_DIM[0]*CHECKERBOARD_DIM[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)*CHECKER_WIDTH


        #Loop twice, right first then left
        for i in range(2):
            calibration_params_path=params_path[i]
            checkerboard_frames_path=frames_path[i]
            if i==0:
                mtx=self.mtx_right
                dist=self.dst_right
            elif i==1:
                mtx=self.mtx_left
                dist=self.dst_left
            
            #Loops for each checkerboard that we previously captured
            for frame_num in range(self.frame_number):
                filename=checkerboard_frames_path+"frame_right"+str(frame_num)+".jpg"
                img = cv2.imread(filename) # Capture frame-by-frame
                gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_DIM, None)
                
                if ret: #Chessboard found, find the rvecs and tvecs, convert to homogeneous and update our lists
                    corners2=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
                    success,rotation_vector,translation_vector,_=cv2.solvePnPRansac(objp,corners2,mtx,dist,\
                                                                                    iterationsCount=RANSAC_SCENE_ITERATIONS,reprojectionError=RANSAC_SCENE_REPROJECTION_ERROR,flags=cv2.USAC_MAGSAC)

                    #Convert rotation_vector and translation_vector to homogeneous transform
                    cam_T_scene=self.convertRvecTvectoHomo(rotation_vector,translation_vector)
                    scene_T_cam=utils.invHomogeneous(cam_T_scene)
                    
                    #Get the frame number to index the corresponding ecm_T_psm pose
                    #name,ext=filename.split('.')   #Splits off the name
                    #frame_num=[int(s) for s in name if s.isdigit()]
                    #combined_number = int("".join(str(number) for number in frame_num))
                    ecm_T_psm=self.ecm_T_psm1_list[frame_num]

                    #Updating the lists
                    if i==0: #right
                        ecm_T_psm3_right.append(ecm_T_psm)
                        scene_T_rightcam.append(scene_T_cam)
                    elif i==1:
                        ecm_T_psm3_left.append(ecm_T_psm)
                        scene_T_leftcam.append(scene_T_cam)


        #Solving Hand-Eye Calibration for both the right and left cameras
        
        #Do Right First
        A=[]
        B=[]
        for i in range(len(ecm_T_psm3_right)-1):
            A_i=np.dot(np.transpose(scene_T_rightcam[i]),scene_T_rightcam[i+1])
            B_i=np.dot(ecm_T_psm3_right[i],np.transpose(ecm_T_psm3_right))
            A.append(A_i)
            B.append(B_i)
        
        #Solve hand-eye problem
        A=np.array(A)
        B=np.array(B)
        rightcam_T_ecm=self.hand_eye.ComputeHandEye(A,B)
        data_right = {'rightcam_T_ecm': rightcam_T_ecm.tolist()}

        #Do Left Next
        A=[]
        B=[]
        for i in range(len(ecm_T_psm3_left)-1):
            A_i=np.dot(np.transpose(scene_T_leftcam[i]),scene_T_leftcam[i+1])
            B_i=np.dot(ecm_T_psm3_left[i],np.transpose(ecm_T_psm3_left))
            A.append(A_i)
            B.append(B_i)
        
        #Solve hand-eye problem
        A=np.array(A)
        B=np.array(B)
        leftcam_T_ecm=self.hand_eye.ComputeHandEye(A,B)
        data_left = {'leftcam_T_ecm': leftcam_T_ecm.tolist()}

        #Store these values
        with open(calibration_params_right+"hand_eye_calibration_right.yaml","w") as f:
                yaml.dump(data_right,f)

        with open(calibration_params_left+"hand_eye_calibration_left.yaml","w") as f:
                yaml.dump(data_left,f)           


    def getFolderName(self):
        #Returns overall root folder where we store the calibration parameters
        if self.rootName is None:
            root_directory=CALIBRATION_DIR
            folder_count=1
            while True:
                if os.path.isdir(root_directory):
                    folder_count_str=str(folder_count)
                    root_directory=root_directory+'Calib_'+folder_count_str
                    folder_count+=1
                else:
                    break
            os.mkdir(root_directory)
            self.rootName=root_directory

    def convertRvecTvectoHomo(self,rvec,tvec):
        Rot,_=cv2.Rodrigues(rvec)
        transform=np.identity(4)
        transform[0:3,0:3]=Rot
        transform[0:3,3]=tvec
        return transform
    


if __name__=='__main__':
    rospy.init_node('CameraCalibrator')
    rospy.Rate(10000)
   
    GUI=CameraCalibGUI()
    cv2.destroyAllWindows()

