import numpy as np
import math


class HandEye:
    #This is for solution to classic hand-eye problem of AX=XB
    # B= tip to camera
    # A=tip to endoscope
    # X=transform from camera to endoscope 

    #def __init__(self):

    def ComputeHandEye(self,A,B):
        #Takes in B and A (B is tool tip to camera, A is tool tip to endoscope)
        #B and A are an N array of 4x4 matrix where N is the number of transforms collected
        #Returns: X the transform from camera to endoscope

        #Note that A and B are arrays of arrays, N=length of outer array=number of transformations
        #A and B must be the same lengths (N)

        N=len(A)
        if N>=2:
            print("A: "+str(A[0]))
            a,a_prime=self.HomoToDualQuat(A[0])
            b,b_prime=self.HomoToDualQuat(B[0])
            L=self.contructK(a,b) #Starts the "L" matrix
            L_prime=self.contructK(a_prime,b_prime)

            #Construct the L matrix for SVD decomposition to solve for 
            for i in range(1,N):
                a,a_prime=self.HomoToDualQuat(A[i])
                b,b_prime=self.HomoToDualQuat(B[i])
                L=np.append(L,self.contructK(a,b))
                L_prime=np.append(L_prime,self.contructK(a_prime,b_prime))
            print("L: "+str(L))
            print("L_prime: "+str(L_prime))

        else:
            print("Not Enough Transforms")
            return None


                



    def contructK(self,a,b):
        #Takes the real part of two dual quaternions to contruct the K matrix to build the L matrix for SVD decomposition
        #See reference: Robust Hand-Eye Calibration for Computer Aided Medical Endoscopy
        #Assume input is two numpy arrays

        a0=a[0] #Real part of quaternion
        a_bar=a[1:4]    #Complex part of quaternion

        b0=b[0] #Real part of quaternion
        b_bar=b[1:4]    #Complex part of quaternion


        a_plus_b=np.add(a_bar,b_bar)
        a_minus_b=np.subtract(a_bar,b_bar)
        bottom_right=np.add(self.SkewSymmetricMatrix(a_plus_b),(a0-b0)*np.identity(3))

        K_mat=np.zeros((4,4))

        K_mat[1:4,1:4]=bottom_right
        K_mat[0,0]=a0-b0
        K_mat[0,1:4]=-(a_bar-b_bar)
        K_mat[1:4,0]=a_bar-b_bar


        return K_mat
    
    def HomoToDualQuat(self,A):
        #Takes in a homogenous transform matrix, A, and converts it to a dual quaternion
        #A is a 4x4 numpy array (homogeneous transform)
        #Returns: a,a'(the real/dual parts of the dual quaternion)
        col_ind=[0,1,2]
        tolerance=0.05


        RA=A[0:3,0:3] #Rotation of A
        tA=A[0:3,3] #Translation of A
        
        #Enforce orthogonality
        RA=self.EnforceOrthogonality(RA)
        print("RA: "+str(RA))
        #########Find the real part of the dual quaternion

        #Extract axis and angle of rotation (we use eigenvector/eigenvalue decomposition)
        eigvals,eigvecs=np.linalg.eig(RA)
        print("eigvals"+str(eigvals))
        ind_axis = np.where(np.logical_and(np.real(eigvals) < (1 + tolerance), np.real(eigvals) > (1 - tolerance)))
        rotation_axis=np.real(eigvecs[:,ind_axis[0][0]])
        del col_ind[ind_axis[0][0]]
        rotation_angle=np.angle(eigvals[col_ind[0]]) #Look at this, not quite right

        #Real part of dual quaternion
        a=np.hstack([np.sin(rotation_angle/2),np.cos(rotation_angle/2)*rotation_axis])

        ############Find the dual part of the dual quaternion
        m=0.5*(np.cross(tA,rotation_axis)+(1/np.tan(rotation_angle/2))*np.cross(rotation_axis,np.cross(tA,rotation_axis)))
        d=np.dot(tA,rotation_axis)
        a_prime=np.hstack([-(d/2)*np.sin(rotation_angle/2),np.sin(rotation_angle/2)*m+(d/2)*np.cos(rotation_angle/2)*rotation_axis])


        return a,a_prime

    def SkewSymmetricMatrix(self,v):
        #Takes the skew symmetric matrix of a vector
        skew_mat=np.array([[0,-v[2],v[1]],
                          [v[2],0,-v[0]],
                          [-v[1],v[0],0]])       
        
        return skew_mat


    def EnforceOrthogonality(self,R):
        #Function which enforces a rotation matrix to be orthogonal
        #R is a 3x3 numpy array

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


    


