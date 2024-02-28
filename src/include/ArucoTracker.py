import cv2
import cv2.aruco as aruco



class ArucoTracker:

    def __init__(self,app):
        #Init Aruco Marker things
        self.aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_1000) #using the 4x4 dictionary to find markers
        self.aruco_params=aruco.DetectorParameters()
        self.arco_detector=aruco.ArucoDetector(self.aruco_dict,self.aruco_params)

        self.app=app
    
    def arucoTracking(self,left_right):
        if left_right=='left':
            corners_left,ids_left,rejected=self.arco_detector.detectMarkers(self.app.frame_left_converted)

            if len(corners_left) > 0:
                self.app.frame_left_converted=aruco.drawDetectedMarkers(self.app.frame_left_converted,corners_left,ids_left)
                #print(ids_left)

        if left_right=='right':
            corners_right,ids_right,rejected=self.arco_detector.detectMarkers(self.app.frame_right_converted)

            if len(corners_right) > 0:
                self.app.frame_right_converted=aruco.drawDetectedMarkers(self.app.frame_right_converted,corners_right,ids_right)
                #print(ids_right)

