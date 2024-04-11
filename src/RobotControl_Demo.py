#Demo for reading/writing to the dVRK joints
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
import crtk



if __name__ == '__main__':
    rospy.init_node('dvrk_arm_test', anonymous=True)
    print('testing arms')
    ecm=dvrk.ecm("ecm")
    print(ecm.enable())
    print(ecm.home())
    goal=ecm.measured_cp()
    print(goal)
    
    #goal.p[2]-=0.05
    #psm1.move_cp(goal).wait()
    