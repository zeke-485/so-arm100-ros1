#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
def main():
    rospy.init_node('zero_pose')
    pub = rospy.Publisher('/so_100_arm_controller/command', JointTrajectory, queue_size=1, latch=True)
    rospy.sleep(1.0)
    joint_names = ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']
    msg = JointTrajectory()
    msg.joint_names = joint_names
    p = JointTrajectoryPoint()
    p.positions = [0.0]*6
    p.time_from_start = rospy.Duration(2.0)
    msg.points = [p]
    pub.publish(msg)
    rospy.loginfo("Published zero pose")
if __name__=='__main__':
    main()
