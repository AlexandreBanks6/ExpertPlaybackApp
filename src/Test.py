import numpy as np

'''
CHECKERBOARD_DIM=(8,8)
objp = np.zeros((CHECKERBOARD_DIM[0]*CHECKERBOARD_DIM[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD_DIM[0], 0:CHECKERBOARD_DIM[1]].T.reshape(-1, 2)*0.0079248  #I think this is already the object point size as it should be

print(objp)
'''
name='frame_right12'
frame_num=[int(s) for s in name if s.isdigit()]
combined_number = int("".join(str(number) for number in frame_num))
print(combined_number)