import tkinter as tk
import glm
import numpy as np
import dvrk
import rospy
from include import utils
import tf_conversions.posemath as pm
import PyKDL

ecm=dvrk.ecm("ECM")
ecm.enable()
ecm.home()

#Create Motion Matrices:
frame_list=[]
#1
#Rotate
R=utils.rotationX(2*(np.pi/180))
R=R*utils.rotationY(3*(np.pi/180))
frame=np.eye(4)
frame[:3,:3]=R
#Translate
frame[:3,3]=np.array([0,0.005,0],dtype='float32')
frame_list.append(frame)

#2
#Rotate
R=utils.rotationZ(2.5*(np.pi/180))
R=R*utils.rotationY(6*(np.pi/180))
frame=np.eye(4)
frame[:3,:3]=R
#Translate
frame[:3,3]=np.array([0.008,0,0],dtype='float32')
frame_list.append(frame)

#3
#Rotate
R=utils.rotationX(-2*(np.pi/180))
R=R*utils.rotationZ(-6*(np.pi/180))
frame=np.eye(4)
frame[:3,:3]=R
#Translate
frame[:3,3]=np.array([0,0,0.01],dtype='float32')
frame_list.append(frame)

#4
#Rotate
R=utils.rotationY(-4*(np.pi/180))
R=R*utils.rotationX(-3*(np.pi/180))
frame=np.eye(4)
frame[:3,:3]=R
#Translate
frame[:3,3]=np.array([0.005,0,-0.012],dtype='float32')
frame_list.append(frame)

#5
#Rotate
R=utils.rotationY(2*(np.pi/180))
R=R*utils.rotationZ(-2*(np.pi/180))
frame=np.eye(4)
frame[:3,:3]=R
#Translate
frame[:3,3]=np.array([0,0.001,0.02],dtype='float32')
frame_list.append(frame)

#6
#Rotate
# R=utils.rotationX(10*(np.pi/180))
# R=R*utils.rotationY(8*(np.pi/180))
# frame=np.eye(4)
# frame[:3,:3]=R
# #Translate
# frame[3,:3]=np.array([0,0.05,0],dtype='float32')
# frame_list.append(frame)

def convertNumpyFrameToPyKDL(frame_numpy):
    rotation_matrix=frame_numpy[:3,:3]
    translation_vector=frame_numpy[:3,3]
    kdl_rotation=PyKDL.Rotation(
        rotation_matrix[0,0],rotation_matrix[0,1],rotation_matrix[0,2],
        rotation_matrix[1,0],rotation_matrix[1,1],rotation_matrix[1,2],
        rotation_matrix[2,0],rotation_matrix[2,1],rotation_matrix[2,2]
    )
    kdl_translation=PyKDL.Vector(translation_vector[0],translation_vector[1],translation_vector[2])
    kdl_frame=PyKDL.Frame(kdl_rotation,kdl_translation)
    return kdl_frame

rospy.sleep(1)



for i in range(0,len(frame_list)):
    
    #Gets starting ECM pose
    cart_T_ecmi=ecm.setpoint_cp()
    cart_T_ecmi=utils.enforceOrthogonalPyKDL(cart_T_ecmi)
    cart_T_ecmi=utils.convertPyDK_To_GLM(cart_T_ecmi)


    #Moves the ECM
    desired_motion=frame_list[i] #Motion in Numpy
    desired_motion_glm=glm.mat4(*desired_motion.T.flatten())
    print("Desired Motion: "+str(desired_motion_glm))
    desired_pose=cart_T_ecmi*desired_motion_glm
    desired_pose=np.array(glm.transpose(desired_pose).to_list(),dtype='float32')
    desired_pose=convertNumpyFrameToPyKDL(desired_pose)
    ecm.move_cp(desired_pose).wait()
    rospy.sleep(2)
    
    #Gets current ECM pose
    cart_T_ecmcurr=ecm.setpoint_cp()    
    cart_T_ecmcurr=utils.enforceOrthogonalPyKDL(cart_T_ecmcurr)
    cart_T_ecmcurr=utils.convertPyDK_To_GLM(cart_T_ecmcurr)


    #Calculates motion
    ecmi_T_ecmcurr=utils.invHomogeneousGLM(cart_T_ecmi)*cart_T_ecmcurr
    print("Actual Motion: "+str(ecmi_T_ecmcurr))

    ecmi_T_ecmcurr=np.array(glm.transpose(ecmi_T_ecmcurr).to_list(),dtype='float32')
    angle_diff,translation_diff=utils.decomposed_difference(ecmi_T_ecmcurr,desired_motion)
    angle_diff=angle_diff*(180/np.pi)
    print("angle_diff: "+str(angle_diff))
    print("translation_diff: "+str(translation_diff))



    rospy.sleep(4)



'''
CHECKERBOARD_DIM=(8,8)
objp = np.zeros((CHECKERBOARD_DIM[0]*CHECKERBOARD_DIM[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)*0.0079248  #I think this is already the object point size as it should be

print(objp)
'''
'''
name='frame_right12'
frame_num=[int(s) for s in name if s.isdigit()]
combined_number = int("".join(str(number) for number in frame_num))
print(combined_number)
'''
'''
class TestGUI:
    def __init__(self):
        self.gui_window=tk.Tk()
        self.gui_window.title("dVRK Playback App")

        self.gui_window.rowconfigure([0,1,2,3,4,5],weight=1)
        self.gui_window.columnconfigure([0,1,2],weight=1)
        self.gui_window.minsize(300,100)

        #Title at top
        self.welcome_text=tk.Label(self.gui_window,text="Welcome to the dVRK Playback App")
        self.welcome_text.grid(row=0,column=1,sticky='n')

        #Calibrate Scene Button
        self.calibrate_scene_button=tk.Button(self.gui_window,text="Calibrate Scene")
        self.calibrate_scene_button.grid(row=1,column=0,sticky="nsew")

        #Select PSM (to record/playback) Checkboxes
        self.checkbox_PSM1=tk.Checkbutton(self.gui_window,text="PSM1",onvalue=1,offvalue=0)
        self.checkbox_PSM1.grid(row=1,column=1,sticky='nsew')

        self.checkbox_PSM3=tk.Checkbutton(self.gui_window,text="PSM3",onvalue=1,offvalue=0)
        self.checkbox_PSM3.grid(row=1,column=2,sticky='nsew')

        #Calibrate Gaze Button
        self.calibrate_gaze_button=tk.Button(self.gui_window,text="Calibrate Gaze")
        self.calibrate_gaze_button.grid(row=2,column=0,sticky="nsew")

        #Playback (render) Tools Button
        self.render_button=tk.Button(self.gui_window,text="Playback Tools")
        self.render_button.grid(row=2,column=1,sticky="nsew")

        #Record Expert Motions Button
        self.record_motions_button=tk.Button(self.gui_window,text="Record Motions")
        self.record_motions_button.grid(row=3,column=0,sticky="nsew")

        #Playback the ECM Motion Button (indicates on the screen arrows showing direction for ECM movement)
        self.playback_ecm_button=tk.Button(self.gui_window,text="Playback ECM Motion")
        self.playback_ecm_button.grid(row=3,column=1,sticky="nsew")

        #Message box to send text to the user
        self.message_box = tk.Text(self.gui_window, height=10, width=80)
        self.message_box.config(state='disabled')
        self.message_box.grid(row=4, column=0, columnspan=3, sticky='nsew', padx=10, pady=10)
        self.displayMessage("Ensure")

        #Button for NDI Validation
        self.ndi_toggle_button=tk.Button(self.gui_window,text="Start NDI Tracker and Validate")
        self.ndi_toggle_button.grid(row=5,column=0,sticky="nsew")

        self.gui_window.mainloop()


    def displayMessage(self,message):
        #function to insert messages in the text box
        self.message_box.config(state='normal')
        self.message_box.delete("1.0",tk.END)

        self.message_box.insert(tk.END, message + "\n")
        self.message_box.see(tk.END)

        self.message_box.config(state='disabled')


test_gui=TestGUI()
'''
'''
glm_matrix=glm.mat4(1)
numpy_array = np.array(glm_matrix.to_list())
print(numpy_array)

glm_matrix=glm.mat4(*numpy_array.flatten())
print(glm_matrix)
'''