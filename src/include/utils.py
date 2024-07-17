##File with utility functions for converting between transformation types, enforcing orthogonality, and converting from opencv vectors  to
#homogeneous transforms

import numpy as np
import glm
import tf_conversions.posemath as pm
import cv2
from scipy.spatial.transform import Rotation



def invHomogeneousNumpy(transform):
    #Input: Homogeneous transform (numpy)
    #Output: Inverse of homogeneous transform (numpy)
    R=transform[0:3,0:3]

    R_trans=np.transpose(R)
    d=transform[0:3,3]
    neg_R_trans_d=-1*np.dot(R_trans,d)
    inverted_transform=np.identity(4)
    inverted_transform[0:3,0:3]=R_trans
    inverted_transform[0:3,3]=neg_R_trans_d
    return inverted_transform
def invHomogeneousGLM(transform):
    frame_new=np.array(glm.transpose(transform).to_list(),dtype='float32')
    inverted_transform=invHomogeneousNumpy(frame_new)
    inverted_transform=inverted_transform.T
    inverted_transform=glm.mat4(*inverted_transform.flatten())
    return inverted_transform

def convertRvecTvectoHomo(rvec,tvec):
    tvec=tvec.flatten()
    #print("tvec new: "+str(tvec))
    #Input: OpenCV rvec (rotation) and tvec (translation)
    #Output: Homogeneous Transform
    Rot,_=cv2.Rodrigues(rvec)
    #print("Rotation Matrix: "+str(Rot))
    transform=np.identity(4)
    transform[0:3,0:3]=Rot
    transform[0:3,3]=tvec
    transform=EnforceOrthogonalityNumpy_FullTransform(transform)
    return transform

def EnforceOrthogonalityNumpy_FullTransform(transform):
    transform[0:3,0:3]=EnforceOrthogonalityNumpy(transform[0:3,0:3])

    return transform

def EnforceOrthogonalityNumpy(R):
    #Function which enforces a rotation matrix to be orthogonal
    #Input: R is a 4x numpy rotation

    #Extracting columns of rotation matrix
    x=R[:,0] 
    y=R[:,1]
    z=R[:,2]
    diff_err=np.dot(x,y)
    x_orth=x-(0.5*diff_err*y)
    y_orth=y-(0.5*diff_err*x)
    z_orth=np.cross(x_orth,y_orth)
    x_norm=0.5*(3-np.dot(x_orth,x_orth))*x_orth
    y_norm=0.5*(3-np.dot(y_orth,y_orth))*y_orth
    z_norm=0.5*(3-np.dot(z_orth,z_orth))*z_orth
    R_new=np.column_stack((x_norm,y_norm,z_norm))

    return R_new


def enforceOrthogonalPyKDL(pykdl_frame):
    #Takes in a pykdl frame and enforces orthogonality
    pykdl_frame_new=pm.toMatrix(pykdl_frame)
    pykdl_frame_new[0:3,0:3]=EnforceOrthogonalityNumpy(pykdl_frame_new[0:3,0:3])
    pykdl_frame_new=pm.fromMatrix(pykdl_frame_new)
    return pykdl_frame_new


def enforceOrthogonalGLM(GLM_frame):
    #Takes in a pykdl frame and enforces orthogonality
    frame_new=np.array(glm.transpose(GLM_frame).to_list(),dtype='float32')
    frame_new[0:3,0:3]=EnforceOrthogonalityNumpy(frame_new[0:3,0:3])
    frame_new=frame_new.T
    frame_new=glm.mat4(*frame_new.flatten())
    return frame_new



def rotationX(theta):
    #Rotate about x axis by theta
    R=np.array([[1,0,0],
               [0,np.cos(theta),-np.sin(theta)],
               [0,np.sin(theta),np.cos(theta)]],dtype='float32')
    return R

def rotationY(theta):
    #Rotate about x axis by theta
    R=np.array([[1,0,0],
               [0,np.cos(theta),-np.sin(theta)],
               [0,np.sin(theta),np.cos(theta)]],dtype='float32')
    return R

def rotationZ(theta):
    #Rotate about z axis by theta
    R=np.array([[np.cos(theta),0,np.sin(theta)],
               [0,0,0],
               [-np.sin(theta),0,np.cos(theta)]],dtype='float32')
    return R


def SkewSymmetricMatrix(v):
    #Takes the skew symmetric matrix of a vector
    skew_mat=np.array([[0,-v[2],v[1]],
                        [v[2],0,-v[0]],
                        [-v[1],v[0],0]],dtype='float32')       
    
    return skew_mat

def convertPyDK_To_GLM(pykdl_frame):
    numpy_frame=pm.toMatrix(pykdl_frame)
    numpy_frame=numpy_frame.T
    glm_frame=glm.mat4(*numpy_frame.flatten())
    return glm_frame

def convertHomogeneousToCSVROW(transform):
    #Input: 4x4 numpy array for homogeneous transform
    #Output: 12x1 string list with: "Tx","Ty","Tz","R00","R01","R02","R10","R11","R12","R20","R21","R22"

    string_list=[str(transform[0,3]),str(transform[1,3]),str(transform[2,3]),\
                str(transform[0,0]),str(transform[0,1]),str(transform[0,2]),\
                str(transform[1,0]),str(transform[1,1]),str(transform[1,2]),\
                str(transform[2,0]),str(transform[2,1]),str(transform[2,2])]
    
    
    return string_list

def scaleGLMTranform(glm_transform,scale_factor):
    #Input: 4x4 transformation matrix (glm_transform), and a scale factor (scale_factor)
    #Output: 4x4 transformation matrix with the translation part scaled
    glm_transform[3][0]=glm_transform[3][0]*scale_factor
    glm_transform[3][1]=glm_transform[3][1]*scale_factor
    glm_transform[3][2]=glm_transform[3][2]*scale_factor

    return glm_transform

def convertAffineToHomogeneous(affine_matrix):
    R=affine_matrix[:,:3]
    t=affine_matrix[:,3]

    U,S,Vt=np.linalg.svd(R)
    R_normalized=np.dot(U,Vt)

    transform=np.eye(4)
    transform[:3,:3]=R_normalized
    transform[:3,3]=t
    transform=EnforceOrthogonalityNumpy_FullTransform(transform)

    return transform

def decomposed_difference(A, B):
    # Extract rotation matrices and translation vectors
    RA, RB = A[:3, :3], B[:3, :3]
    tA, tB = A[:3, 3], B[:3, 3]
    translation_diff = np.linalg.norm(tA - tB)

    # Convert rotation matrices to quaternions
    quatA, quatB = Rotation.from_matrix(RA).as_quat(), Rotation.from_matrix(RB).as_quat()

    # Calculate the angular difference between quaternions
    rotation_diff = Rotation.from_quat(quatA).inv() * Rotation.from_quat(quatB)
    angle_diff = rotation_diff.magnitude()

    # Calculate the Euclidean distance between translation vectors
    

    return angle_diff, translation_diff


#################Code to Find Rigid Transform Between Two Point Sets
def estimateRigidTransform(A,B):
    """
    Estimate the rigid transformation between two sets of corresponding 3D points using the Kabsch algorithm.
    :param A: Nx3 array of 3D points in the first coordinate frame.
    :param B: Nx3 array of 3D points in the second coordinate frame.
    :return: Rotation matrix (3x3), translation vector (3,)
    """
    assert A.shape==B.shape,"Input point sets must have the same shape."


    #Compute Centroids
    centroid_A=np.mean(A,axis=0)
    centroid_B=np.mean(B,axis=0)

    #Center the points
    AA=A-centroid_A
    BB=B-centroid_B

    #Compute the Covariance Matrix
    H=np.dot(AA.T,BB)

    #SVD Decomposition
    U,S,Vt=np.linalg.svd(H)
    # d = np.linalg.det(Vt.T @ U.T)
    # e = np.array([[1, 0, 0], [0, 1, 0], [0, 0, d]])

    # R = Vt.T @ e @ U.T
    # t=centroid_B-np.matmul(R,centroid_A)

    R=np.dot(Vt.T,U.T)

    
    #Correct for Reflection
    if np.linalg.det(R)<0:
        Vt[-1,:]*=-1
        R=np.dot(Vt.T,U.T)

    t=centroid_B-np.dot(R,centroid_A)
    
    return R,t

def ransacRigidRransformation(A,B,num_iterations=1000,distance_threshold=0.005):
    """
    Estimate the rigid transformation using RANSAC.
    :param A: Nx3 array of 3D points in the first coordinate frame.
    :param B: Nx3 array of 3D points in the second coordinate frame.
    :param num_iterations: Number of RANSAC iterations.
    :param distance_threshold: Distance threshold for inliers.
    :return: 4x4 homogeneous transformation matrix inliers (boolean mask)
    """

    best_inliers = None
    best_R, best_t = None, None
    num_points = A.shape[0]
    best_num_inliers = 0

    for _ in range(num_iterations):

        #randomly select subset of 3 points
        indices=np.random.choice(num_points,3,replace=False)
        A_subset=A[indices]
        B_subset=B[indices]

        #Estimate the transform using the subset
        try:
            R_candidate,t_candidate=estimateRigidTransform(A_subset,B_subset)
        except np.linalg.LinAlgError:
            continue

        #Apply the transformation to all the points in A
        A_transformed=(R_candidate@A.T).T+t_candidate

        #Compute the distances to the corresponding points in B
        distances=np.linalg.norm(A_transformed-B,axis=1)

        #Find inliers
        inliers=distances<distance_threshold
        num_inliers=np.sum(inliers)

        if num_inliers>best_num_inliers:
            best_num_inliers=num_inliers
            best_R,best_t=R_candidate,t_candidate
            best_inliers=inliers
    
    full_transform=np.eye(4)
    full_transform[:3,:3]=best_R
    full_transform[:3,3]=best_t
    full_transform=EnforceOrthogonalityNumpy_FullTransform(full_transform)

    return full_transform,best_inliers

def convertHomoToRvecTvec_GLM(transform_glm):
    transform_np=np.array(glm.transpose(transform_glm).to_list(),dtype='float32')
    R=transform_np[:3,:3]
    t=transform_np[:3,3]

    rvec,_=cv2.Rodrigues(R)
    tvec=t.reshape((3,1))
    


    return rvec,tvec