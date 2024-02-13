import cv2
import os

curr_frame_number=1
for i in range(1,2803):
    full_path='../chessboard_images_left_raw/frame'+str(i)+".jpg"
    print(full_path)
    raw_frame=cv2.imread(full_path)
    cv2.imshow("Frame",raw_frame)
    k=cv2.waitKey(0)
    if k==ord('t'):
        print("t")
        full_path='../chessboard_images_left_calib/frame'+str(curr_frame_number)+".jpg"
        cv2.imwrite(full_path,raw_frame)
        curr_frame_number+=1
    
    elif k==ord('e'):
        break
    else:
        continue