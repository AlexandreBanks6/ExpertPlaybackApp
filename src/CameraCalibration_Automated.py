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

##################Change These Variables If Needed#############################
CHECKERBOARD_DIM=(8,8) #Number of inner corners in the checkerboard (corners height, corners width)
REQUIRED_CHECKERBOARD_NUM=10 #Number of checkerboard images needed for the calibration
PROPORTIONAL_GAIN=0.00001 #Gain for centering ECM above checkerboard
ERROR_THRESHOLD=10 #Pixel Error Threshold for centering ECM above checkerboard

#Where the images for the calibration are saved

CALIBRATION_DIR="../resources/Calib" #Where we store calibration parameters and images


RIGHT_FRAMES_FILEDIR="/chessboard_images_right/"
LEFT_FRAMES_FILEDIR="/chessboard_images_left/"

#Where the calibration parameters are saved
RIGHT_CAMERA_CALIB_DIR="/calibration_params_right/"
LEFT_CAMERA_CALIB_DIR="/calibration_params_left/"

#Ros Topics
RightFrame_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftFrame_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'


#20 Preset Motions from Starting Pose

'''
Sets of 5 motions for 4 different z axis increments:
Firt, increment z
then:
1. No rotation/Translation
2. Rotate about z by positive angle
3. Rotate about z by negative angle
4. Translate along x by positive val
5. translate along y by positive val
Repeat 4 times

We define these as numpy frames, they are later converted to PyKDL frames

'''
def rotationZ(theta):
    R=np.array([[np.cos(theta),-np.sin(theta),0],
               [np.sin(theta),np.cos(theta),0],
               [0,0,1]])
    return R


z_translation=0.025 #Amount that we translate along z (2.5 centimeters)
planar_translation=0.05 #Amount that we translate in plane parallel to endoscope
rotation_angle=20*(np.pi/180) #20 degrees in Rad

#T1, no change
T1=np.identity(4)
print("T1: "+str(T1))
#T2 rotation about z positive
Rz=rotationZ(rotation_angle)
Rz=HandEye.EnforceOrthogonality(Rz)
T2=np.identity(4)
T2[0:3,0:3]=Rz
print("T2: "+str(T2))
#T3 rotation about z negative
Rz=rotationZ(-rotation_angle)
Rz=HandEye.EnforceOrthogonality(Rz)
T3=np.identity(4)
T3[0:3,0:3]=Rz
print("T3: "+str(T3))

#T4 translation along x
T4=np.identity(4)
T4[0,3]=planar_translation
print("T4: "+str(T4))
#T4 translation along y
T5=np.identity(4)
T5[1,3]=planar_translation
print("T5: "+str(T5))
MOTIONS=[T1,T2,T3,T4,T5]


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
        #Checkerboard subpix criteria:
        self.criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


        #Execution Loop  
        #Window where we display the left/right frames:
        cv2.namedWindow("Left/Right Frames",cv2.WINDOW_NORMAL)
        
        self.window.after(1,self.showFramesCallback)
        self.window.mainloop()
    
    def showFramesCallback(self):
        #We move servo the ECM along plane until its center aligns with the center of the checkerboard


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
        print("Image Center: "+str(image_center))
        checkerboard_center=np.mean(corners,axis=0)
        print("Checkerboard Center: "+str(checkerboard_center))
        e_x=checkerboard_center[0][0] - image_center[0]
        e_y=checkerboard_center[0][1] - image_center[1]
        return e_x,e_y

    def grabFramesCallback(self):
        self.getFolderName()
        file_name_right=self.rootName+RIGHT_FRAMES_FILEDIR
        file_name_left=self.rootName+LEFT_FRAMES_FILEDIR

        #First, servo ECM in 2D (along x,y plane) to have ECM center at checkerboard center (there is no z-axis movement)
        #For now only servo based on left camera
        corners=self.findCheckerboard(self.frameLeft)
        if corners is not None:
            e_x,e_y=self.findAxisErrors(self.frameLeft,corners)
            err_mag=np.sqrt((e_x**2)+(e_y**2))
            x_sign=1
            y_sign=1
            loop_num=1
            while err_mag>ERROR_THRESHOLD: #Loops until center of LEFT ECM is within acceptable threshold
                print("ex: "+str(e_x))
                #print("ey: "+str(e_y))
                move_x=x_sign*PROPORTIONAL_GAIN*e_x #Amount to move along x
                move_y=y_sign*PROPORTIONAL_GAIN*e_y #Amount to move along y
                print("move_x:" +str(move_x))
                #print("move_y:" +str(move_y))
                #Moving ECM
                ecm_pose_curr=self.ecm.setpoint_cp()
                #print("ECM Curr: "+str(ecm_pose_curr))
                ecm_pose_curr.p[0]+=move_x
                ecm_pose_curr.p[1]+=move_y
                #print("ECM New: "+str(ecm_pose_curr))
                #ecm_pose_curr=self.ecm.measured_cp()
                #Transform matrix
                #T=np.identity(4)
                #T=T[0,3]=move_x
                #T=T[1,3]=move_y
                #T=pm.fromMatrix(T)
                #ecm_pose_curr=ecm_pose_curr*T
                self.ecm.move_cp(ecm_pose_curr).wait()
                print("Moved")
                rospy.sleep(0.05)

                #Updating measurements
                print("Finding Corners")
                corners=self.findCheckerboard(self.frameLeft)
                if corners is not None:
                    ex_new,ey_new=self.findAxisErrors(self.frameLeft,corners)
                    '''
                    if loop_num%2==0:
                        if np.abs(ex_new)>np.abs(e_x):
                            x_sign=-x_sign
                    else:
                        if np.abs(ey_new)>np.abs(e_y):
                            y_sign=-y_sign

                    '''


                    '''
                    if loop_num==1:
                        if np.abs(ex_new)>np.abs(e_x):
                            x_sign=-x_sign
                        if np.abs(ey_new)>np.abs(e_y):
                            y_sign=-y_sign
                            loop_num+=1
                    '''

                    e_x=ex_new
                    e_y=ey_new
                    print("ex_new: "+str(e_x))
                    #print("ey_new: "+str(e_y))
                    err_mag=np.sqrt((e_x**2)+(e_y**2))
                else:
                    print("Returned")
                    return
                loop_num+=1
                    


        else:
            return




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
        self.getFolderName()
        frames_right_path=self.rootName+RIGHT_FRAMES_FILEDIR
        frames_left_path=self.rootName+LEFT_FRAMES_FILEDIR

        calibration_params_right=self.rootName+RIGHT_CAMERA_CALIB_DIR
        calibration_params_left=self.rootName+LEFT_CAMERA_CALIB_DIR

        frames_path=[frames_right_path,frames_left_path]
        params_path=[calibration_params_right,calibration_params_left]

        #Repeats Twice, right first then left
        for i in range(0,1):
            calibration_params_path=params_path[i]
            checkerboard_frames_path=frames_path[i]

            if not os.path.isdir(calibration_params_path):
                os.mkdir(calibration_params_path)

            #Camera Calibration params
            objp = np.zeros((CHECKERBOARD_DIM[0]*CHECKERBOARD_DIM[1],3), np.float32)
            objp[:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)*0.0079248

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
            else:
                self.calbration_message_label.config(text="Note enough acceptable checkerboards, grab more frames, need "+str(REQUIRED_CHECKERBOARD_NUM))
                return
            
            #Finding Re-projection error
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error
            data = {'camera_matrix': np.asarray(mtx).tolist(),
                        'dist_coeff': np.asarray(dist).tolist(),
                        'mea reprojection error': [mean_error/len(objpoints)]}
            
            #Writing data to file
            with open(calibration_params_path+"calibration_matrix.yaml","w") as f:
                yaml.dump(data,f)
    def getFolderName(self):
        #Returns overall root folder where we store the calibration parameters
        if self.rootName is None:
            root_directory=CALIBRATION_DIR
            folder_count=2
            while True:
                if os.path.isdir(root_directory):
                    folder_count_str=str(folder_count)
                    root_directory=root_directory[:-1]+folder_count_str
                    folder_count+=1
                else:
                    break
            os.mkdir(root_directory)
            self.rootName=root_directory

        


if __name__=='__main__':
    rospy.init_node('CameraCalibrator')
    rospy.Rate(10000)
   
    GUI=CameraCalibGUI()
    cv2.destroyAllWindows()

