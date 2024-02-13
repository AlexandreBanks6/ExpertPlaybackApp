import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import tkinter as tk
import threading
import os
import yaml


##################Change These Variables If Needed#############################
CHECKERBOARD_DIM=(8,8) #Number of inner corners in the checkerboard (corners height, corners width)
REQUIRED_CHECKERBOARD_NUM=4 #Number of checkerboard images needed for the calibration
#Where the images for the calibration are saved
CALIBRATION_DIR="../resources/Calib1" #Where we store calibration parameters




RIGHT_FRAMES_FILEDIR="/chessboard_images_right/"
LEFT_FRAMES_FILEDIR="/chessboard_images_left/"

#Where the calibration parameters are saved
RIGHT_CAMERA_CALIB_DIR="/calibration_params_right/"
LEFT_CAMERA_CALIB_DIR="/calibration_params_left/"

mutex=threading.Lock()

class CameraCalibGUI:
    frameRight=cv2.Mat
    frameLeft=cv2.Mat
    frame_number=1
    isRight=False
    isLeft=False
    ranOnce=0
    isUserFolder=False
    rootName=None
    def __init__(self):

        self.bridge=CvBridge()
        self.window=tk.Tk()
        self.window.title("dVRK Camera Calibrator")
        self.window.rowconfigure([0,1,2,3],weight=1)
        self.window.columnconfigure([0,1],weight=1)

        #----Message Box
        self.message_label=tk.Label(text="Welcome to the dVRK Endoscope Calibrator",width=40)
        self.message_label.grid(row=0,column=0)

        #Text box for file directory
        self.intput_file=tk.Text(height=1,width=40)
        self.intput_file.grid(row=4,column=1,sticky="nsew")

        #---Buttons
        self.button_frame=tk.Button(text="Grab Frame",width=15,command=self.grabFrameCallback)
        self.button_frame.grid(row=1,column=0,sticky="nsew")

        self.button_calibrate=tk.Button(text="Calibrate",width=15,command=self.calibrationCallback)
        self.button_calibrate.grid(row=2,column=0,sticky="nsew")

        #---Check Boxes
        #self.isRight=tk.IntVar()
        #self.isLeft=tk.IntVar()

        self.checkbox_right_camera=tk.Checkbutton(text="Calibrate Right",width=20,onvalue=1,offvalue=0,command=self.rightCheckbox)
        self.checkbox_right_camera.grid(row=1,column=1,sticky="nsew")
        self.checkbox_left_camera=tk.Checkbutton(text="Calibrate Left",width=20,onvalue=1,offvalue=0,command=self.leftCheckbox)
        self.checkbox_left_camera.grid(row=2,column=1,sticky="nsew")

        self.checkbox_user_file=tk.Checkbutton(text="Provide Location to store Calibration Parameters?",width=50,onvalue=1,offvalue=0,command=self.isUserFolder)
        self.checkbox_user_file.grid(row=3,column=0, sticky="nsew")

        #Initializing Loop

        #print("Started")
        #rightCamera =  message_filters.Subscriber('ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed', CompressedImage)
        #leftCamera = message_filters.Subscriber('ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed', CompressedImage)

        #ts = message_filters.TimeSynchronizer([rightCamera, leftCamera],1)
        #ts.registerCallback(self.frameCallback2)
        #print("Registered")


        keyRight='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
        keyLeft='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'        
        #rospy.Subscriber(name = keyRight, data_class=CompressedImage, callback=self.frameCallback,callback_args='right',queue_size=None)
        #rospy.Subscriber(name = keyLeft, data_class=CompressedImage, callback=self.frameCallback, callback_args='left',queue_size=None)
        rospy.Subscriber(name = keyRight, data_class=CompressedImage, callback=self.frameCallbackRight,queue_size=None)
        rospy.Subscriber(name = keyLeft, data_class=CompressedImage, callback=self.frameCallbackLeft,queue_size=None)
        #rospy.Timer(rospy.Duration(0.3),sticky="nsew",self.showFramesCallback)
        
        #Window where we display the left/right frames:
        cv2.namedWindow("Left/Right Frames",cv2.WINDOW_NORMAL)
        
        self.window.after(1,self.showFramesCallback)
        self.window.mainloop()
    
    def showFramesCallback(self):
        if CameraCalibGUI.ranOnce>=2:
            img_combined=cv2.hconcat([CameraCalibGUI.frameLeft,CameraCalibGUI.frameRight])
            img_combined=cv2.resize(img_combined,(700,246),interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Left/Right Frames",img_combined)
            cv2.waitKey(1)
        self.window.after(1,self.showFramesCallback)


    def frameCallback (self, data, args):
        print(args)
        if(args=='right'):
            CameraCalibGUI.frameRight=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
            CameraCalibGUI.ranOnce+=1

        if(args=='left'):
            CameraCalibGUI.frameLeft=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
            CameraCalibGUI.ranOnce+=1

     
    def frameCallbackRight(self,data):
        #mutex.acquire(blocking=True)
        CameraCalibGUI.frameRight=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        CameraCalibGUI.ranOnce+=1
        #mutex.release()


    def frameCallbackLeft(self,data):
        #mutex.acquire(blocking=True)
        CameraCalibGUI.frameLeft=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        CameraCalibGUI.ranOnce+=1
        #mutex.release()

    def grabFrameCallback(self):
        self.getFolderName()
        file_name_right=CameraCalibGUI.rootName+RIGHT_FRAMES_FILEDIR
        file_name_left=CameraCalibGUI.rootName+LEFT_FRAMES_FILEDIR
        if not os.path.isdir(file_name_right):
            os.mkdir(file_name_right)
            os.mkdir(file_name_left)
        cv2.imwrite(file_name_right+"frame"+str(CameraCalibGUI.frame_number)+".jpg",CameraCalibGUI.frameRight)
        cv2.imwrite(file_name_left+"frame"+str(CameraCalibGUI.frame_number)+".jpg",CameraCalibGUI.frameLeft)
        CameraCalibGUI.frame_number+=1
   
    def rightCheckbox(self):
        CameraCalibGUI.isRight=not CameraCalibGUI.isRight
        print("Callback")

    def leftCheckbox(self):
        CameraCalibGUI.isLeft=not CameraCalibGUI.isLeft

    def userFolderDefined(self):
        CameraCalibGUI.isUserFolder=True

    def calibrationCallback(self):
        self.getFolderName()
        if CameraCalibGUI.isRight:
            self.message_label.config(text="Calibrating Right Camera")
            checkerboard_frames_path=CALIBRATION_DIR+RIGHT_FRAMES_FILEDIR
            #calibration_params_path=CameraCalibGUI.rootName+RIGHT_CAMERA_CALIB_DIR
            calibration_params_path=CALIBRATION_DIR+RIGHT_CAMERA_CALIB_DIR
        elif CameraCalibGUI.isLeft:
            self.message_label.config(text="Calibrating Left Camera")
            checkerboard_frames_path=CALIBRATION_DIR+LEFT_FRAMES_FILEDIR
            #calibration_params_path=CameraCalibGUI.rootName+LEFT_CAMERA_CALIB_DIR
            calibration_params_path=CALIBRATION_DIR+LEFT_CAMERA_CALIB_DIR
        else:
            self.message_label.config(text="Select Which Camera to Calibrate")
            return
        if not os.path.isdir(calibration_params_path):
            os.mkdir(calibration_params_path)
        print("Calibration Started")
        #Camera Calibration params
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((1, CHECKERBOARD_DIM[0] * CHECKERBOARD_DIM[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)

        objpoints=[]
        imgpoints=[]
        found=0
        for file in os.listdir(checkerboard_frames_path):  # Here, 10 can be changed to whatever number you like to choose
            filename=os.fsdecode(file)
            if filename.endswith(".jpg"):
                print(filename)
                path=checkerboard_frames_path+filename
                img = cv2.imread(path) # Capture frame-by-frame
                #print(images[im_i])
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_DIM, None)

                # If found, add object points, image points (after refining them)
                if ret == True:
                    objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
                    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                    imgpoints.append(corners2)
                    # Draw and display the corners
                    img = cv2.drawChessboardCorners(img, CHECKERBOARD_DIM, corners2, ret)
                    found += 1
                    cv2.imshow('Chessboard Frame', img)
                    cv2.waitKey(500)
        
        if found>=REQUIRED_CHECKERBOARD_NUM:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        else:
            self.message_label.config(text="Note enough acceptable checkerboards, grab more frames, need "+str(REQUIRED_CHECKERBOARD_NUM))
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
        if CameraCalibGUI.isUserFolder:
            CameraCalibGUI.rootName=self.intput_file.get("1.0",'end-1c')
        else:
            if CameraCalibGUI.rootName is None:
                root_directory=CALIBRATION_DIR
                print(root_directory)
                folder_count=2
                while True:
                    print("entered loop")
                    if os.path.isdir(root_directory):
                        folder_count_str=str(folder_count)
                        root_directory=root_directory[:-1]+folder_count_str
                        print(root_directory)
                        folder_count+=1
                    else:
                        print("Brok Loop")
                        break
                os.mkdir(root_directory)
                CameraCalibGUI.rootName=root_directory

        


if __name__=='__main__':
    rospy.init_node('ExpertPlayback',anonymous=True)
    rospy.Rate(10000)
    GUI=CameraCalibGUI()
    #GUI.window.mainloop()
    cv2.destroyAllWindows()

