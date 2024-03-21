import glm
import numpy as np
from include import HandEye
def normalize_2d(matrix):
    norm = np.linalg.norm(matrix)
    matrix = matrix/norm  # normalized matrix
    return matrix

hand_eye_obj=HandEye.HandEye()

#a=np.array([0.1,0.2,0.3,0.4])
#b=np.array([0.5,0.6,0.7,0.8])

#hand_eye_obj.contructK(a,b)

#R=np.array([[0.2,0.5,0.3],
 #           [0.8,0.1,0.11],
  #          [-0.2,-0.55,-0.25]])
#R_new=hand_eye_obj.EnforceOrthogonality(R)
#A=np.array([[0.2,0.5,0.3,2],
        #    [0.8,0.1,0.11,5],
        #    [-0.2,-0.55,-0.25,6],
         #   [0,0,0,1]])
#a,a_prime=hand_eye_obj.HomoToDualQuat(A)

A=[normalize_2d(np.random.rand(4,4))]
B=[normalize_2d(np.random.rand(4,4))]
print("A: "+str(A))
print("B: "+str(B))
for i in range(0,3):
    new_A=np.random.rand(4,4)
    new_B=np.random.rand(4,4)

    A.append(normalize_2d(new_A))
    B.append(normalize_2d(new_B))

A = np.array(A)
B = np.array(B)
print("A: "+str(A))
print("B: "+str(B))
hand_eye_obj.ComputeHandEye(A,B)




