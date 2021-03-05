#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from tf.transformations import euler_from_quaternion,quaternion_from_euler

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('grasp_eg', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene._pub_co = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)
    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    moveit_commander.move_group.MoveGroupCommander.set_planner_id(arm_group,"SPARS")
    #putting arma at inital place
    arm_group.set_named_target("allZeros")
    #arm_group.set_planning_time(5)
    plan1 = arm_group.go()
    #going to 1st object i.e. soap
    pose_target_1 = geometry_msgs.msg.Pose()
    
    orientation_1 = quaternion_from_euler(-0.0003,1.4759,-1.5738)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.6427
    pose_target_1.position.y = 0.1001
    pose_target_1.position.z = 0.9378
    arm_group.set_pose_target(pose_target_1)
    plan2 = arm_group.go()

    orientation_1 = quaternion_from_euler(0.8015,1.4759,-1.5744)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.5839
    pose_target_1.position.y = -0.008
    pose_target_1.position.z = 0.9481
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()
    #opening the gripper
    OpenGripper()
    #pre grasping pose
    orientation_1 = quaternion_from_euler(0.8016,1.4759,-1.5742)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.5735
    pose_target_1.position.y = -0.0041
    pose_target_1.position.z = 0.8837
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    #grasping the soap

    orientation_1 = quaternion_from_euler(0.7936,1.4763,-1.5818)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.5656
    pose_target_1.position.y = -0.0030
    pose_target_1.position.z = 0.8330
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    #closing the gripper
    CloseGripper()

    arm_group.set_named_target("straightUp")
    arm_group.set_planning_time(10)
    plan1 = arm_group.go()

    orientation_1 = quaternion_from_euler(0.7984,1.4772,-1.5765)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.5656
    pose_target_1.position.y = 0.0099
    pose_target_1.position.z = 0.9711
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    orientation_1 = quaternion_from_euler(0.7960,1.4771,-1.5784)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.0266
    pose_target_1.position.y = 0.5890
    pose_target_1.position.z = 1.157
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    orientation_1 = quaternion_from_euler(0.7931,1.4759,-1.580)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.0556
    pose_target_1.position.y = 0.7488
    pose_target_1.position.z = 1.0076
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    #release the soap
    OpenGripper()
    #pre grasping pose for biscuit
    orientation_1 = quaternion_from_euler(-0.007,1.5396,-1.5782)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.5620
    pose_target_1.position.y = -0.2251
    pose_target_1.position.z = 0.9146
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()
    #grasping biscuits
    orientation_1 = quaternion_from_euler(-0.005,1.5395,-1.5763)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.5620
    pose_target_1.position.y = -0.2563
    pose_target_1.position.z = 0.8772
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    CloseGripper2()

    orientation_1 = quaternion_from_euler(-0.005,1.5395,-1.5763)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.5620
    pose_target_1.position.y = -0.2563
    pose_target_1.position.z = 0.8772
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    orientation_1 = quaternion_from_euler(-0.5911,1.5383,-1.5740)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.0543
    pose_target_1.position.y = -0.7403
    pose_target_1.position.z = 1.0323
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()
    #release biscuits
    OpenGripper()

    #approaching soap2
    orientation_1 = quaternion_from_euler(2.7790,1.5621,1.6493)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.4573
    pose_target_1.position.y = 0.2307
    pose_target_1.position.z = 0.9134
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()
    #grasping soap2
    orientation_1 = quaternion_from_euler(2.7599,1.5622,1.6300)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.4573
    pose_target_1.position.y = 0.2314
    pose_target_1.position.z = 0.8283
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    CloseGripper2()

    #dropping soap2
    orientation_1 = quaternion_from_euler(-0.5911,1.5383,-1.5740)
    pose_target_1.orientation.x = orientation_1[0]
    pose_target_1.orientation.y = orientation_1[1]
    pose_target_1.orientation.z = orientation_1[2]
    pose_target_1.orientation.w = orientation_1[3]

    pose_target_1.position.x = 0.0543
    pose_target_1.position.y = -0.7403
    pose_target_1.position.z = 1.0323
    arm_group.set_pose_target(pose_target_1)
    
    plan2 = arm_group.go()

    OpenGripper()

    arm_group.set_named_target("straightUp")
    plan1 = arm_group.go()

    rospy.sleep(2)
    moveit_commander.roscpp_shutdown()


def OpenGripper():
    #opening the gripper
    hand_group  = moveit_commander.MoveGroupCommander("gripper")
    hand_group.set_named_target("open")
    plan2 = hand_group.go()
    

def CloseGripper():
    #closing the gripper
    hand_group  = moveit_commander.MoveGroupCommander("gripper")
    hand_group.set_named_target("gripper close")
    plan3 = hand_group.go()

def CloseGripper2():
    #closing the gripper
    hand_group  = moveit_commander.MoveGroupCommander("gripper")
    hand_group.set_named_target("close2")
    plan4 = hand_group.go()

if __name__ == '__main__':
    main()