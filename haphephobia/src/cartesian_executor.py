#!/usr/bin/env python3
import sys
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger
from haphephobia.srv import generate_cartesian_plan
import copy
import time

class PathSegment (object):
    def __init__ (self):
        self.cartesian_success = None
        self.cartesian_plan = None
        self.initial_joint_state = None
        self.terminal_joint_state = None
        self.viable_start_pose = True

class CartesianPlanner (object):
    
    # - Construtor
    def __init__ (self):
        rospy.init_node('region_bounded_allowance_planner')
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        self.robot_name = rospy.get_param('/robot/model')
        group_name = rospy.get_param('/robot/manipulation/planning_group')
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.marker_frame = rospy.get_param('/robot/frames/root_link')
        self.eef_frame = rospy.get_param('/robot/frames/eef_link')
        self.named_stow_config = rospy.get_param('/robot/manipulation/named_stow_config')
        
        self.move_group.set_planning_time(1.0) # ! Remember this is here
        
        self.success_marker_pub = rospy.Publisher("/surface_repair/markers/fixture_success", MarkerArray, queue_size=1)
        spray_pub_topic = rospy.get_param('/robot/virtual_fixtures/tool_tip/repair_spray_topic')
        self.spray_pub = rospy.Publisher(spray_pub_topic, Bool, queue_size=1)
        
        self.cartesian_generation = rospy.Service('cartesian_pose_plan', generate_cartesian_plan, self.cartesianPlan)
        self.cartesian_execution = rospy.Service('cartesian_plan_execution', Trigger, self.cartesianExecute)
        
        self.cartesian_plan = None
        
    # - Helper to make sure the robot is at the desired state
    def checkIfAtJointState (self, target, tol=0.1):
        state = self.move_group.get_current_joint_values()
        res = max(abs(target[idx] - state[idx]) for idx in range(len(state)))
        if res > tol:
            return False
        return True
    
    def checkIfAtPose (self, target, c_tol=0.01, a_tol=0.05):
        pose = self.move_group.get_current_pose()
        if (abs(pose.pose.position.x - target.position.x) > c_tol) or (abs(pose.pose.position.y - target.position.y) > c_tol) or (abs(pose.pose.position.z - target.position.z) > c_tol) or (abs(pose.pose.orientation.w - target.orientation.w) > a_tol) or (abs(pose.pose.orientation.x - target.orientation.x) > a_tol) or (abs(pose.pose.orientation.y - target.orientation.y) > a_tol) or (abs(pose.pose.orientation.z - target.orientation.z) > a_tol):
            return False
        return True
    
    def cartesianDistance (self, p1:Pose, p2:Pose) -> float:
        return np.sqrt((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2 + (p1.position.z - p2.position.z)**2)
        
    def calculatePathLength (self, poses:list) -> float:
        cumulative_distance:float = 0
        for i in range(len(poses) - 1):
            # - Calculate length between each set of two poses, add it to running total
            cumulative_distance += self.cartesianDistance(poses[i], poses[i+1])
        return cumulative_distance
    
    # - Generator for Cartesian plans
    def cartesianPlan (self, msg) -> bool:
        if len(msg.input_poses) <= 1:
            # TODO gracefully handle lists of just one pose 
            raise NotImplementedError

        # - Start by going to the first pose
        # - Need to avoid situation where the first pose is unreachable and thus we fail altogether
        found_valid_start = False
        start_index = 0
        while not found_valid_start:
            if start_index > len(msg.input_poses):
                rospy.logwarn('Failed to find a valid starting pose in the pose list; this SHOULD indicate that ALL poses are unreachable. If this is not the case, investigate.')
                return False
            self.move_group.set_pose_target(msg.input_poses[start_index])
            success = self.move_group.plan()
            if success[0]:
                found_valid_start = True
                self.move_group.go(wait=True)
            else:
                start_index += 1
        
        remaining_list = msg.input_poses[start_index:]
        print(f'Length of remaining list: {len(remaining_list)}, first {len(msg.input_poses) - len(remaining_list)} poses removed because they were unreachable.')
        
        # - Plan to lists
        while len(remaining_list) > 1:
            remaining_list, plan = self.growPath(remaining_list, 0.2)
            
            # - Execute plan with spray commands
            spray_msg = Bool()
            spray_msg.data = True
            if plan is not None:
                self.spray_pub.publish(spray_msg)
                self.move_group.execute(plan)
            else:
                self.move_group.go(wait=True)
                self.spray_pub.publish(spray_msg)
                time.sleep(1.0)
            spray_msg.data = False
            self.spray_pub.publish(spray_msg)
            self.move_group.stop()
            
            # - Small check to make sure we arrived at the pose we were supposed to
            if not self.checkIfAtPose(msg.input_poses[len(msg.input_poses[start_index:]) - len(remaining_list)]):
                rospy.loginfo('Robot appears to be at the wrong terminal pose; trying to plan non-cartesian-ally to get it there.')
                found_next_viable_pose = False
                offset_index = 0
                while not found_next_viable_pose:
                    rospy.loginfo(f'Trying to find the next valid pose, offset index is {offset_index}')
                    # - Handle condition when I run out of poses
                    if (offset_index > len(remaining_list)):
                        rospy.loginfo("Failed to find another reachable pose in the list. Returning.")
                        self.move_group.set_named_target(self.named_stow_config)
                        success = self.move_group.plan()
                        if success[0]:
                            self.move_group.go(wait=True)
                        return True
                        
                    self.move_group.set_pose_target(msg.input_poses[len(msg.input_poses) - (len(remaining_list) + offset_index)])
                    success = self.move_group.plan()
                    if success:
                        rospy.loginfo(f'Found next viable pose at offset index {offset_index}')
                        found_next_viable_pose = True
                        # - Reduce the size of remaining list accordingly
                        remaining_list = remaining_list[offset_index:]
                        self.move_group.go(wait=True)
                    else:
                        offset_index += 1
        self.move_group.set_named_target(self.named_stow_config)
        success = self.move_group.plan()
        if success[0]:
            self.move_group.go(wait=True)
        return True
    
    # - Outputs of this are a list of poses yet to operate on and a plan for executing the ones we removed, need to call this recursively until the poses list has no length
    def growPath (self, poses:list, length_limit=None) -> tuple:
        input_pose_list_length = len(poses)
        stored_plan = None
        remaining_list = poses
        
        for i in range(len(poses)):
            subposes = [poses[i] for i in range(i + 1)]
            
            # - checking path length
            path_length = self.calculatePathLength(subposes)
            if length_limit is not None:
                if path_length > length_limit:
                    rospy.logdebug(f'Returning because path length {path_length} exceeds limit of {length_limit}.')
                    # & Handling case where no plan can be completed that is under the defined limit
                    if len(remaining_list) == len(poses):
                        rospy.logdebug('Length of remaining list is same and input list!')
                        self.move_group.set_pose_target(poses[i])
                        test = self.move_group.plan()
                        return remaining_list[i:], None # ? returning None plan here so we know Cartesian chaining failed
                    return remaining_list, stored_plan
            
            # - trying to plan
            (plan, fraction) = self.move_group.compute_cartesian_path(
                subposes, 0.01, 0.0
            )
            
            # - seeing how successful the plan is
            if fraction < 1:
                print(fraction)
                # TODO add in failure mode where pose is skipped if we fail to plan to it on the first one
                if len(remaining_list) == input_pose_list_length:
                    rospy.logwarn("Found an infeasible starting pose; continuing execution from next pose in list")
                    remaining_list = remaining_list[1:]
                return remaining_list, stored_plan
            
            stored_plan = copy.deepcopy(plan)
            remaining_list = poses[i:]
            
        return remaining_list, stored_plan
        
    # - Spinner   
    def run (self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    cartesian_planner = CartesianPlanner()
    cartesian_planner.run()
