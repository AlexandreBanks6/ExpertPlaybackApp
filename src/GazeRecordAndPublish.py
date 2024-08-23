import cv2
import rospy

MAX_FRAME_FAIL_COUNT=50 #If we fail to grab 50 frames consecutively, stop running





##Get User Input for Video Channel (default 0)
device_number=input("Input Gaze Video Decize Number (default 0): ")

#Sets up Video Capture
cap=cv2.VideoCapture(int(device_number))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)

#Checks if the device is available
if not cap.isOpened:
    print("Error: Could not open video device " +str(device_number)+", try again")
    exit()

######Init Counters######
frame_fail_count=0
frame_count=0

while True:
    ret,frame=cap.read()

    if not ret:
        frame_fail_count+=1
        if frame_fail_count>=MAX_FRAME_FAIL_COUNT:
            print("Gaze Tracker Not Grabbing Frames")
            break
    else:
        frame_fail_count=0 #Resets the failed frame grabs counter

        gray_chan,_,_=cv2.split(frame)  #Extracts the grayscale frame
        frame_count+=1 #Incrementing the frame counter

        
        
        cv2.imshow("Gaze Video: ",gray_chan) #Showing image
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()