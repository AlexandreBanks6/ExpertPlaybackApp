import numpy as np

def decomposed_difference(A, B):
    # Extract rotation matrices and translation vectors
    RA, RB = A[:3, :3], B[:3, :3]
    tA, tB = A[:3, 3], B[:3, 3]

    # Convert rotation matrices to quaternions
    quatA, quatB = Rotation.from_matrix(RA).as_quat(), Rotation.from_matrix(RB).as_quat()

    # Calculate the angular difference between quaternions
    rotation_diff = Rotation.from_quat(quatA).inv() * Rotation.from_quat(quatB)
    angle_diff = rotation_diff.magnitude()

    # Calculate the Euclidean distance between translation vectors
    translation_diff = np.linalg.norm(tA - tB)

    return angle_diff, translation_diff


A1=np.array([[0.62556,0.197539,0.754754,-0.0148933],
             [-0.702114,0.56,0.754754,-0.0148933],
             [0.62556,0.197539,0.754754,-0.0148933],
             [0.62556,0.197539,0.754754,-0.0148933],],dtype='float32')

