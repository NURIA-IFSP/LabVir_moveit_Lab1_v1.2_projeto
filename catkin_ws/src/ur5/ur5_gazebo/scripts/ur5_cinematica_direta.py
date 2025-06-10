#!/usr/bin/python
#
# Unified UR5 and Gripper Control with specific gripper methods
#

import rospy
import actionlib
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class UR5WithGripperControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ur5_gripper_control')
        
        # Setup joint trajectory publisher
        self.joint_pub = rospy.Publisher('/trajectory_controller/command',
                                       JointTrajectory,
                                       queue_size=10)
        
        # Setup gripper action client
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/gripper_cmd',
            control_msgs.msg.GripperCommandAction
        )
        self.gripper_client.wait_for_server()
        
    def move_to_joint_positions(self, positions, duration=2.0):
        """Move UR5 to specified joint positions"""
        traj = JointTrajectory()
        traj.header = Header()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                           'elbow_joint', 'wrist_1_joint', 
                           'wrist_2_joint', 'wrist_3_joint']
        
        pts = JointTrajectoryPoint()
        pts.positions = positions
        pts.time_from_start = rospy.Duration(duration)
        
        traj.points = [pts]
        self.joint_pub.publish(traj)
        rospy.sleep(duration)  # Wait for movement to complete
        
    def control_gripper(self, position, max_effort=-1.0):
        """Control gripper position (0.0=open, 0.8=closed)"""
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        return self.gripper_client.get_result()
    
    def open_gripper(self, max_effort=-1.0):
        """Fully open the gripper"""
        rospy.loginfo("Opening gripper...")
        return self.control_gripper(0.0, max_effort)
    
    def close_gripper(self, max_effort=-1.0):
        """Fully close the gripper"""
        rospy.loginfo("Closing gripper...")
        return self.control_gripper(0.8, max_effort)

def main():
    controller = UR5WithGripperControl()
    
    # Define positions (in radians)
    # Position A: Home position
    pos_a = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    
    # Position B: Example position
    pos_b = [0.5, -1.0, 1.0, -1.0, -1.57, 0.5]
    
    try:
        # Move to position A
        rospy.loginfo("Moving to position A...")
        controller.move_to_joint_positions(pos_a)
        
        # Close gripper (pick object)
        controller.close_gripper()
        
        # Move to position B
        rospy.loginfo("Moving to position B...")
        controller.move_to_joint_positions(pos_b)
        
        # Open gripper (release object)
        controller.open_gripper()
        
        rospy.loginfo("Movement sequence completed!")
        
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted before completion")

if __name__ == '__main__':
    main()