import tkinter as tk
import glm
import numpy as np
import dvrk
import rospy
from include import utils

psm1=dvrk.psm("PSM1") #Mapped to left hand

psm1.enable()
psm1.home()
rospy.sleep(1)



while(1):
    #joint_vars_psm1=psm1.measured_js()[0]
    #jaw_angle_psm1=psm1.jaw.measured_js()[0]
    ecm_T_psm=psm1.measured_cp()
    #print("Joint Vars: "+str(joint_vars_psm1*(180/np.pi)))
    #print("Original Pose: "+str(ecm_T_psm))
    ecm_T_psm=utils.enforceOrthogonalPyKDL(ecm_T_psm)
    ecm_T_psm=utils.convertPyDK_To_GLM(ecm_T_psm)
    print("Pose New: "+str(ecm_T_psm))
    rospy.sleep(3)



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