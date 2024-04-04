#Demo for reading/writing to the dVRK joints
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
import crtk



if __name__ == '__main__':
    rospy.init_node('dvrk_arm_test', anonymous=True)
    print('testing arms')
    psm1=dvrk.psm("PSM1")
    print(psm1.enable())
    print(psm1.home())
    goal=psm1.setpoint_cp()
    
    goal.p[2]-=0.05
    psm1.move_cp(goal).wait()
    