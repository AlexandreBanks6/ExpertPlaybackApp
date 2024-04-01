#Demo for reading/writing to the dVRK joints
from include import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm



if __name__ == '__main__':

    print('testing arms')
    psm1=dvrk.psm("PSM1")
    psm1.enable()
    psm1.home()