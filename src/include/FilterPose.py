<<<<<<< HEAD
import numpy as np
import glm
from include import utils
from scipy.spatial.transform import Rotation



class FilterPose:
    def __init__(self,filter_length=8):
        self.list_quaternions=[]    #List of 1x4 rows with the quaternion
        self.list_translation=[] #List of 1x3 rows with the x,y,z translation
        self.filter_length=filter_length

    def appendHomoToFilter(self,transform):
        #Transform is a numpy array

        #Extract Quaternion
        R=transform[:3,:3] #Extracts rotation part
        quat=Rotation.from_matrix(R).as_quat() #Converts it to a quaternion form (x,y,z,w)
        quat=quat.tolist()

        #Extracts the translation part
        trans=transform[:3,3]
        trans=trans.tolist()

        #Append to buffers
        self.list_quaternions.append(quat)
        self.list_translation.append(trans)
        if len(self.list_quaternions)>self.filter_length:
            self.list_quaternions.pop(0) #Gets rid of earliest added element if buffer length is met
        
        if len(self.list_translation)>self.filter_length:
            self.list_translation.pop(0) #Gets rid of earliest added element if buffer length is met


    def findFilterMean(self):
        #Returns the mean homogeneous transform in glm format

        #Mean quaternion:
        quaternion_array=np.array(self.list_quaternions)
        mean_quat=np.mean(quaternion_array,axis=0)

        #Mean translation:
        translation_array=np.array(self.list_translation)
        mean_trans=np.mean(translation_array,axis=0)


        #Construct homogeneous transform
        R=Rotation.from_quat(mean_quat).as_matrix()

        homo_transform=np.eye(4)
        homo_transform[:3,:3]=R
        homo_transform[:3,3]=mean_trans

        #Converts to glm transform
        glm_transform=glm.mat4(*homo_transform.T.flatten())
        glm_transform=utils.enforceOrthogonalGLM(glm_transform)

        return glm_transform

=======
import numpy as np
import glm
from include import utils
from scipy.spatial.transform import Rotation



class FilterPose:
    def __init__(self,filter_length=8):
        self.list_quaternions=[]    #List of 1x4 rows with the quaternion
        self.list_translation=[] #List of 1x3 rows with the x,y,z translation
        self.filter_length=filter_length

    def appendHomoToFilter(self,transform):
        #Transform is a numpy array

        #Extract Quaternion
        R=transform[:3,:3] #Extracts rotation part
        quat=Rotation.from_matrix(R).as_quat() #Converts it to a quaternion form (x,y,z,w)
        quat=quat.tolist()

        #Extracts the translation part
        trans=transform[:3,3]
        trans=trans.tolist()

        #Append to buffers
        self.list_quaternions.append(quat)
        self.list_translation.append(trans)
        if len(self.list_quaternions)>self.filter_length:
            self.list_quaternions.pop(0) #Gets rid of earliest added element if buffer length is met
        
        if len(self.list_translation)>self.filter_length:
            self.list_translation.pop(0) #Gets rid of earliest added element if buffer length is met


    def findFilterMean(self):
        #Returns the mean homogeneous transform in glm format

        #Mean quaternion:
        quaternion_array=np.array(self.list_quaternions)
        mean_quat=np.mean(quaternion_array,axis=0)

        #Mean translation:
        translation_array=np.array(self.list_translation)
        mean_trans=np.mean(translation_array,axis=0)


        #Construct homogeneous transform
        R=Rotation.from_quat(mean_quat).as_matrix()

        homo_transform=np.eye(4)
        homo_transform[:3,:3]=R
        homo_transform[:3,3]=mean_trans

        #Converts to glm transform
        glm_transform=glm.mat4(*homo_transform.T.flatten())
        glm_transform=utils.enforceOrthogonalGLM(glm_transform)

        return glm_transform

>>>>>>> cb2dc77b51fe54e55eb17648b0794c2400e6f547
