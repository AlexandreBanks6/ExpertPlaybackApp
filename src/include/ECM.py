
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
import trimesh
from trimesh.visual import texture, TextureVisuals
import pyrender
from PIL import Image

#PyOpenGL Imports
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import matplotlib.cm
from vectors import *
from math import *


CONSOLE_VIEWPORT_WIDTH=1024
CONSOLE_VIEWPORT_HEIGHT=722

class ECM_Class:
    left_frame=None

    ###Things for Rendering###
    #left_mesh=None
    #scene_left=pyrender.Scene()
    def __init__(self):
        #Init cv-ROS bridge object
        self.bridge=CvBridge()

        #Init Aruco Marker things
        self.aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_1000) #using the 4x4 dictionary to find markers
        self.aruco_params=aruco.DetectorParameters()
        self.arco_detector=aruco.ArucoDetector(self.aruco_dict,self.aruco_params)

        ############Loding Models for Rendering#################
        shaft_trimesh=trimesh.load("../resources/instrument_models/shaft.ply")
        self.shaft_mesh=pyrender.Mesh.from_trimesh(shaft_trimesh)

        body_trimesh=trimesh.load("../resources/instrument_models/logobody.ply")
        self.body_mesh=pyrender.Mesh.from_trimesh(body_trimesh)

        right_jaw_trimesh=trimesh.load("../resources/instrument_models/jawRight.ply")
        self.sharight_jaw_mesh=pyrender.Mesh.from_trimesh(right_jaw_trimesh)

        left_jaw_trimesh=trimesh.load("../resources/instrument_models/jawLeft.ply")
        self.left_jaw_mesh=pyrender.Mesh.from_trimesh(left_jaw_trimesh)
        #################Renering Initializers####################


        '''
        # Set camera parameters
        camera_pose = np.array([
            [1, 0, 0, 0],  # Right vector
            [0, 1, 0, 0],  # Up vector
            [0, 0, 1, 5],  # Forward vector (distance from the scene)
            [0, 0, 0, 1]   # Translation
        ])
        camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0)
        ECM_Class.scene_left.add(camera, pose=camera_pose)
        print("started scene")
        pyrender.Viewer(ECM_Class.scene_left,viewport_size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT),\
                        use_raymond_lighting=True,run_in_thread=True)
        
        #pyrender.Viewer(self.scene_left,run_in_thread=True,use_raymond_lighting=True)
        print("Started Viewer")

        ############Vertices for Rendering the Background Image###########

        # Create vertices for a rectangle
        self.background_vertices = np.array([
            [-1, -1, 0],  # bottom-left
            [1, -1, 0],   # bottom-right
            [1, 1, 0],    # top-right
            [-1, 1, 0],   # top-left
        ])

        # Create corresponding texture coordinates
        self.background_texcoords = np.array([
            [0, 0],
            [1, 0],
            [1, 1],
            [0, 1]
        ])

        # Define triangle faces using vertices
        self.background_faces = np.array([
            [0, 1, 2],
            [0, 2, 3]
        ])

        '''

    def frameCallback(self,data):
        ECM_Class.left_frame=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.renderScene(ECM_Class.left_frame)
        #cv2.imshow("Frame",cv_image)

        #self.arucoTracking(cv_image)
        #cv_image=bridge.imgmsg_to_cv2(data,'passthrough')


    def arucoTracking(self,frame):
        corners,ids,rejected=self.arco_detector.detectMarkers(frame)
        if len(corners) > 0:
            frame=aruco.drawDetectedMarkers(frame,corners,ids)
        cv2.imshow("Image",frame)
        cv2.waitKey(1)

            # loop over the detected ArUCo corners
        '''
            for (markerCorner, markerID) in zip(corners, ids_new):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners_new = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_new
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                #cv2.putText(frame, str(markerID),
                 #   (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                  #  0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                # show the output image
                cv2.imshow("Image", frame)
                cv2.waitKey(1)
            '''
    def renderScene(self,frame_opencv):
        #if ECM_Class.left_mesh is not None: #Removes the previously rendered background
         #   ECM_Class.scene_left.remove_node(ECM_Class.left_mesh)
        cv2.imshow("Frame",frame_opencv)
        #print(frame_opencv.shape)
        cv2.waitKey(0)



        '''
        background_image=cv2.cvtColor(frame_opencv,cv2.COLOR_BGR2RGB)
        texture = pyrender.Texture(
            source=background_image,
            source_channels="RGB"
        )
        

        background_mesh = pyrender.Mesh.from_trimesh(
            trimesh.Trimesh(
                vertices=self.background_vertices,
                faces=self.background_faces,
                vertex_normals=np.zeros_like(self.background_vertices),
                face_normals=np.zeros_like(self.background_faces),
                texture=texture
            )
        )

        ECM_Class.scene_left.add(background_mesh,pose=np.eye(4))
        '''
        
        '''
        
        background_image=cv2.cvtColor(frame_opencv,cv2.COLOR_BGR2RGB)
        
        background_image=Image.fromarray(background_image)
        background_material=texture.SimpleMaterial(image=background_image)
        background_texture=TextureVisuals(image=background_image,material=background_material)
        ECM_Class.left_mesh = trimesh.Trimesh(
                    vertices=self.background_vertices,
                    faces=self.background_faces,
                    face_normals=np.zeros_like(self.background_faces),
                    visual=background_texture,
                    validate=True,
                    process=False
                )
        '''
        #print("Render Scene")
        #quad=trimesh.Trimesh(vertices=self.background_vertices,faces=self.background_faces,\
        #                    face_normals=np.zeros_like(self.background_faces),\
        #                      visual=background_texture,validate=True,process=False)
        #quad=trimesh.Trimesh(vertices=self.background_vertices,faces=self.background_faces,\
         #                    face_normals=np.zeros_like(self.background_faces),\
          #                      visual=background_texture,validate=True,process=False)
        
        #self.scene_left.add(pyrender.Mesh.from_trimesh(ECM_Class.left_mesh))


        #s = np.sqrt(2)*3/4
        #body_pose=np.array([[0.0, -s,   s,   0.3],[1.0,  0.0, 0.0, 0.0],[0.0,  s,   s,   0.35],[0.0,  0.0, 0.0, 1.0]])

        #scene=pyrender.Scene()
        #scene.add(self.body_mesh,pose=body_pose)
        #pyrender.Viewer(scene,viewport_size=(CONSOLE_VIEWPORT_WIDTH,CONSOLE_VIEWPORT_HEIGHT),\
         #                use_raymond_lighting=True)