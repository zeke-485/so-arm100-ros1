#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64

def send_initial_pose():
    rospy.init_node('send_initial_pose')

    # Publisher for arm controller trajectory command
    arm_pub = rospy.Publisher('/so_100_arm_controller/command', JointTrajectory, queue_size=10)
    # Publisher for gripper position command
    gripper_pub = rospy.Publisher('/so_100_gripper_controller/command', Float64, queue_size=10)

    rospy.sleep(1.0)  # Wait for publishers to be ready

    # Prepare arm trajectory message
    jt = JointTrajectory()
    jt.joint_names = [
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll'
    ]

    # All joint positions set to 0
    point = JointTrajectoryPoint()
    point.positions = [0.0] * len(jt.joint_names)
    point.time_from_start = rospy.Duration(2.0)  # 2 seconds to reach position

    jt.points = [point]

    rospy.loginfo("Sending initial zero position to arm joints...")
    arm_pub.publish(jt)

    rospy.sleep(0.5)  # small delay before gripper command

    rospy.loginfo("Sending initial zero position to gripper...")
    gripper_pub.publish(Float64(0.0))

    rospy.sleep(3.0)  # Give time for commands to be processed

if __name__ == '__main__':
    try:
        send_initial_pose()
    except rospy.ROSInterruptException:
        pass
