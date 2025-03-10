import numpy as np
import math
from include import utils
from scipy.spatial.transform import Rotation
from sklearn import datasets, linear_model

class HandEye:
    #This is for solution to classic hand-eye problem of AX=XB
    # B= tip to camera
    # A=tip to endoscope
    # X=transform from camera to endoscope 

    def __init__(self):
        print('welcome to the Hand Eye Class')

    def ComputeHandEye(self,A,B):
        #Takes in A and B
        #B and A are an N array of 4x4 matrix where N is the number of motions collected
        #Returns: X the homogeneous transform from camera to endoscope (l,c_T_e)

        #Note that A and B are arrays of arrays, N=length of outer array=number of transformations
        #A and B must be the same lengths (N)
        #print("A: "+str(A))
        #print("B: "+str(B))

        N=len(A)
        if N>=2:
            a,a_prime=self.HomoToDualQuat(A[0])
            print("a: "+str(a))
            print("a_prime: "+str(a_prime))
            b,b_prime=self.HomoToDualQuat(B[0])
            L=self.constructK(a,b) #Starts the "L" matrix
            L_prime=self.constructK(a_prime,b_prime)

            #Construct the L matrix for SVD decomposition to solve for 
            for i in range(1,N):
                a,a_prime=self.HomoToDualQuat(A[i])
                b,b_prime=self.HomoToDualQuat(B[i])
                L=np.vstack((L,self.constructK(a,b)))
                L_prime=np.vstack((L_prime,self.constructK(a_prime,b_prime)))

            #Compute rotation part (real quaternion) of hand-eye transform using SVD decomposition
            q=self.solveHomoSVD(L)

            #Compute dual part of the dual quaternion
            A=L #Correct
            B=-1*np.dot(L_prime,q)
            ransac=linear_model.RANSACRegressor(max_trials=10000,residual_threshold=0.025)
            #ransac=RANSACRegressor(base_estimator=base_estimator,min_samples=10,max_trials=1000,residual_threshold=0.1)

            #q_prime,residuals,_,_=np.linalg.lstsq(A,B,rcond=None)
            ransac.fit(A,B)
            q_prime=ransac.estimator_.coef_
            #print("q_prime: "+str(q_prime))
            #print("q_prime_new: "+str(q_prime_new))
            #Solve translation part:
            q_conj=np.array([q[0]]+list(-1*q[1:4]))
            #print("q: "+str(q
            
            #print("q_conj: "+str(q_conj))
            mult_res=self.quaternionMultiply(2*q_prime,q_conj)
            trans=mult_res[1:4]

            #Get the rotation part:
            R=Rotation.from_quat([q[1],q[2],q[3],q[0]])
            R=R.as_matrix()

            #Build the homogeneous transform matrix
            X=np.identity(4)
            X[0:3,0:3]=R
            X[0:3,3]=trans


            result_verifier=np.linalg.norm(np.dot(L,q_prime)+np.dot(L_prime,q))
            print("result_verifier: "+str(result_verifier))

            return X

            
        else:
            print("Not Enough Transforms")
            return None
               

    def quaternionMultiply(self,q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return np.array([w, x, y, z])
    def solveHomoSVD(self,L):
        U,S,Vh=np.linalg.svd(L)
        #print("L: "+str(L))
        #print("Singular Values: "+str(S))
        #print("Vh Values: "+str(Vh))
        return_q=Vh[:,-1]
        #print("return_q: "+str(return_q))
        #The solution "S" or the singular values are sorted in decending order, so we just take the 
        #final column of Vh
        #print("Test val: "+str(np.dot(L,return_q)))


        return return_q

    def constructK(self,a,b):
        #Takes the real part of two dual quaternions to construct the K matrix to build the L matrix for SVD decomposition
        #See reference: Robust Hand-Eye Calibration for Computer Aided Medical Endoscopy
        #Assume input is two numpy arrays
        a0=a[0] #Real part of quaternion
        a_bar=a[1:4]    #Complex part of quaternion

        b0=b[0] #Real part of quaternion
        b_bar=b[1:4]    #Complex part of quaternion


        a_plus_b=np.add(a_bar,b_bar)
        a_minus_b=np.subtract(a_bar,b_bar)
        bottom_right=np.add(utils.SkewSymmetricMatrix(a_plus_b),(a0-b0)*np.identity(3))

        K_mat=np.zeros((4,4))

        K_mat[1:4,1:4]=bottom_right
        K_mat[0,0]=a0-b0
        K_mat[0,1:4]=-1*(a_minus_b)
        K_mat[1:4,0]=a_minus_b


        return K_mat
    '''
    def HomoToDualQuat(self,A):
        #Takes in a homogenous transform matrix, A, and converts it to a dual quaternion
        #A is a 4x4 numpy array (homogeneous transform)
        #Returns: a,a'(the real/dual parts of the dual quaternion)
        RA=A[0:3,0:3] #Rotation of A
        tA=A[0:3,3] #Translation of A
        RA=utils.EnforceOrthogonalityNumpy(RA)
        RA_scipy=Rotation.from_matrix(RA)
        r_vec_scipy=RA_scipy.as_rotvec() #Returns rotation axis and angle encoded
        rotation_angle=np.linalg.norm(r_vec_scipy)
        rotation_axis=r_vec_scipy/rotation_angle if rotation_angle!=0 else r_vec_scipy
        
        w=np.cos(rotation_angle/2)
        x,y,z=np.sin(rotation_angle/2)*rotation_axis
        a=np.array([w,x,y,z])

        trans_quat = np.array([0] + list(tA))  # Translation as a quaternion
        print("trans_quat: "+str(trans_quat))
        a_prime = 0.5 * self.quaternionMultiply(trans_quat,a)
        return a,a_prime
    '''

    def HomoToDualQuat(self,A):
        #Takes in a homogenous transform matrix, A, and converts it to a dual quaternion
        #A is a 4x4 numpy array (homogeneous transform)
        #Returns: a,a'(the real/dual parts of the dual quaternion)
        RA=A[0:3,0:3] #Rotation of A
        tA=A[0:3,3] #Translation of A
        RA=utils.EnforceOrthogonalityNumpy(RA)
        RA_scipy=Rotation.from_matrix(RA)
        r_vec_scipy=RA_scipy.as_rotvec() #Returns rotation axis and angle encoded
        rotation_angle=np.linalg.norm(r_vec_scipy)
        rotation_axis=r_vec_scipy/rotation_angle if rotation_angle!=0 else r_vec_scipy
        
        w=np.cos(rotation_angle/2)
        x,y,z=np.sin(rotation_angle/2)*rotation_axis
        a=np.array([w,x,y,z])

        m=0.5*(np.cross(tA,rotation_axis)+np.cross(rotation_axis,np.cross(tA,rotation_axis))*(1/np.tan(rotation_angle/2)))
        d=tA@rotation_axis
        a_prime=np.hstack([(-d/2)*np.sin(rotation_angle/2),np.sin(rotation_angle/2)*m+(d/2)*np.cos(rotation_angle/2)*rotation_axis])

        return a,a_prime

    


    
    
    '''
    def HomoToDualQuat(self,A):
        #Takes in a homogenous transform matrix, A, and converts it to a dual quaternion
        #A is a 4x4 numpy array (homogeneous transform)
        #Returns: a,a'(the real/dual parts of the dual quaternion)
        RA=A[0:3,0:3] #Rotation of A
        tA=A[0:3,3] #Translation of A
        RA=utils.EnforceOrthogonalityNumpy(RA)

        #We convert the rotation to rotation axis and rotation angle using scipy
        RA_scipy=Rotation.from_matrix(RA)
        r_vec_scipy=RA_scipy.as_rotvec() #Returns rotation axis and angle encoded
        rotation_angle=np.linalg.norm(r_vec_scipy)
        rotation_axis=r_vec_scipy/rotation_angle if rotation_angle!=0 else r_vec_scipy

        #Real part of dual quaternion
        w=np.sin(rotation_angle/2)
        x,y,z=np.cos(rotation_angle/2)*rotation_axis
        a=np.array([w,x,y,z])

        m=0.5*(np.cross(tA,rotation_axis)+(1/np.tan(rotation_angle/2))*np.cross(rotation_axis,np.cross(tA,rotation_axis)))
        d=np.dot(tA,rotation_axis)
        a_prime=np.hstack([-(d/2)*np.sin(rotation_angle/2),np.sin(rotation_angle/2)*m+(d/2)*np.cos(rotation_angle/2)*rotation_axis])
        return a,a_prime

    '''


    '''
    def HomoToDualQuat(self,A):
        #Takes in a homogenous transform matrix, A, and converts it to a dual quaternion
        #A is a 4x4 numpy array (homogeneous transform)
        #Returns: a,a'(the real/dual parts of the dual quaternion)
        col_ind=[0,1,2]
        tolerance=0.05


        RA=A[0:3,0:3] #Rotation of A
        tA=A[0:3,3] #Translation of A
        
        #Enforce orthogonality
        RA=utils.EnforceOrthogonalityNumpy(RA)
        #########Find the real part of the dual quaternion

        #Extract axis and angle of rotation (we use eigenvector/eigenvalue decomposition)
        eigvals,eigvecs=np.linalg.eig(RA)
        #Find index of eigenvector that is only real by finding imaginary parts and part that has zero imaginary part
        imag_parts=np.imag(eigvals)
        ind_only_real=np.where(np.logical_and(imag_parts<tolerance,imag_parts>-tolerance)) #Index of the column with only a real value
        only_real=np.real(eigvals[ind_only_real[0][0]]) #Value of only real part
        if (only_real < (1 + tolerance)) and (only_real > (1 - tolerance)): #Real part is 1
            #ind_real=np.where(np.logical_and(real_parts)
            #ind_axis = np.where(np.logical_and(np.real(eigvals) < (1 + tolerance), np.real(eigvals) > (1 - tolerance)))
            rotation_axis=np.real(eigvecs[:,ind_only_real[0][0]])
            del col_ind[ind_only_real[0][0]]
            rotation_angle=np.angle(eigvals[col_ind[0]]) #Look at this, not quite right

            #Real part of dual quaternion
            a=np.hstack([np.sin(rotation_angle/2),np.cos(rotation_angle/2)*rotation_axis])

            ############Find the dual part of the dual quaternion
            m=0.5*(np.cross(tA,rotation_axis)+(1/np.tan(rotation_angle/2))*np.cross(rotation_axis,np.cross(tA,rotation_axis)))
            d=np.dot(tA,rotation_axis)
            a_prime=np.hstack([-(d/2)*np.sin(rotation_angle/2),np.sin(rotation_angle/2)*m+(d/2)*np.cos(rotation_angle/2)*rotation_axis])
            return a,a_prime
        else:
            print("Not a Rotation Matrix")
            return None,None
        '''
        

##############Maybe use this method instead
'''
def homogeneous_to_dual_quat(transform):
    # Extract rotation matrix and translation vector
    rotation_matrix = transform[0:3, 0:3]
    translation_vector = transform[0:3, 3]

    # Convert rotation matrix to quaternion (real part)
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()  # In the format [x, y, z, w]

    # Compute dual part of the dual quaternion
    # The formula is 0.5 * (translation_vector * real_quaternion)
    trans_quat = np.array([0] + list(translation_vector))  # Translation as a quaternion
    dual_part = 0.5 * quaternion_multiply(trans_quat, np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]]))

    # Combine real and dual parts
    dual_quaternion = np.concatenate((quaternion, dual_part))

    return dual_quaternion
        
        
        '''
        



    


