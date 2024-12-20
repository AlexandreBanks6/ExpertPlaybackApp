#Demo for reading/writing to the dVRK joints
import dvrk
import PyKDL
import rospy
import tf_conversions.posemath as pm
import crtk



if __name__ == '__main__':
    rospy.init_node('CameraCalibrator')
    rospy.Rate(10000)
    rospy.sleep(1)
    ecm=dvrk.ecm("ECM")
    print(ecm.enable())
    print(ecm.home())
    rospy.sleep(1)
    pose=ecm.measured_cp()
    print(pose)

    
    #goal.p[2]-=0.05
    #psm1.move_cp(goal).wait()
    #rospy.sleep(1)
    #psm1=dvrk.psm("PSM1")
    #psm1.enable()
    #psm1.home()
    #pose=psm1.measured_cp()
    