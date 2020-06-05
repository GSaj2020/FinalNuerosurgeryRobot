#!/usr/bin/env python

import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import sys
import copy
import moveit_commander
import moveit_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform
from std_msgs.msg import String
from math import pi
from moveit_commander.conversions import pose_to_list
from ros_igtl_bridge.msg import igtlpoint
from ros_igtl_bridge.msg import igtlstring
from ros_igtl_bridge.msg import igtltransform

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setupok
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('igtl_importer',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()
 

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.
    group_name = "ur5"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

 
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def cb_point(self, data):

    if data.name == 'Target':
        rospy.loginfo(" Target = (%f, %f, %f).", data.pointdata.x, data.pointdata.y, data.pointdata.z)
        self.x_target = data.pointdata.x
        self.y_target = data.pointdata.y
        self.z_target = data.pointdata.z

    elif data.name == 'Entry':
        rospy.loginfo(" Entry = (%f, %f, %f).", data.pointdata.x, data.pointdata.y, data.pointdata.z)
        self.x_entry = data.pointdata.x
        self.y_entry = data.pointdata.y
        self.z_entry = data.pointdata.z

  def igtl_importer(self):

    global pub_igtl_transform_out
    
    pub_igtl_transform_out = rospy.Publisher('IGTL_TRANSFORM_OUT', igtltransform, queue_size=10)
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('igtl_importer', anonymous=True)
    rospy.Subscriber("IGTL_POINT_IN", igtlpoint, self.cb_point)

  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0

    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def RotationTo2Quaternion(self,t):
        #   Adapted from "robotic toolbox for python"
        #   Convert homogeneous transform to a unit-quaternion
        #   Return a unit quaternion corresponding to the rotational part of the
        #   homogeneous transform t.
        #   source: https://blenderartists.org/t/make-quaternion-from-3-vectors/541828/7


        qs = np.sqrt(np.trace(t)+1)/2.0
        kx = t[2,1] - t[1,2]    # Oz - Ay
        ky = t[0,2] - t[2,0]    # Ax - Nz
        kz = t[1,0] - t[0,1]    # Ny - Ox
        if (t[0,0] >= t[1,1]) and (t[0,0] >= t[2,2]):
            kx1 = t[0,0] - t[1,1] - t[2,2] + 1      # Nx - Oy - Az + 1
            ky1 = t[1,0] + t[0,1]           # Ny + Ox
            kz1 = t[2,0] + t[0,2]           # Nz + Ax
            add = (kx >= 0)
        elif (t[1,1] >= t[2,2]):
            kx1 = t[1,0] + t[0,1]           # Ny + Ox
            ky1 = t[1,1] - t[0,0] - t[2,2] + 1  # Oy - Nx - Az + 1
            kz1 = t[2,1] + t[1,2]           # Oz + Ay
            add = (ky >= 0)
        else:
            kx1 = t[2,0] + t[0,2]           # Nz + Ax
            ky1 = t[2,1] + t[1,2]           # Oz + Ay
            kz1 = t[2,2] - t[0,0] - t[1,1] + 1  # Az - Nx - Oy + 1
            add = (kz >= 0)
        if add:
            kx = kx + kx1
            ky = ky + ky1
            kz = kz + kz1
        else:
            kx = kx - kx1
            ky = ky - ky1
            kz = kz - kz1
        kv = np.matrix([kx, ky, kz])
        nm = np.linalg.norm( kv )
        if nm == 0:
            e0 = 1.0
            q = np.matrix([0.0, 0.0, 0.0])
        else:
            e0 = qs
            q = (np.sqrt(1 - qs**2) / nm) * kv
        return e0, q[0,0], q[0,1], q[0,2]

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    ENTRY = np.array([self.x_entry/1000, self.y_entry/1000, self.z_entry/1000])
    TARGET = np.array([self.x_target/1000, self.y_target/1000, self.z_target/1000])

    ENTRY2TARGET = TARGET - ENTRY
    ENTRY2TARGET = ENTRY2TARGET/(np.sqrt(ENTRY2TARGET[0]**2+ENTRY2TARGET[1]**2+ENTRY2TARGET[2]**2))

    V1 = [0.0,1.0,0.0]
    VX = np.cross(V1,ENTRY2TARGET)
    VY = np.cross(ENTRY2TARGET,VX)
    orientation = np.mat([[VX[0],VY[0],ENTRY2TARGET[0]],[VX[1],VY[1],ENTRY2TARGET[1]],[VX[2],VY[2],ENTRY2TARGET[2]]])
    w,x,y,z = self.RotationTo2Quaternion(orientation)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = x
    pose_goal.orientation.y = y
    pose_goal.orientation.z = z
    pose_goal.orientation.w = w
    pose_goal.position.x = self.x_entry/1000
    pose_goal.position.y = self.y_entry/1000
    pose_goal.position.z = self.z_entry/1000
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the yplan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##

    ENTRY = [self.x_entry/1000, self.y_entry/1000, self.z_entry/1000]
    TARGET = [self.x_target/1000, self.y_target/1000, self.z_target/1000]
    MIDPOINT = [(ENTRY[0]+TARGET[0])/2, (ENTRY[1]+TARGET[1])/2, (ENTRY[2]+TARGET[2])/2]

    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x = ENTRY[0]
    wpose.position.y = ENTRY[1]
    wpose.position.z = ENTRY[2]
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = MIDPOINT[0]
    wpose.position.y = MIDPOINT[1]
    wpose.position.z = MIDPOINT[2]
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = TARGET[0]
    wpose.position.y = TARGET[1]
    wpose.position.z = TARGET[2]
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


def main():
  try:
    
    MOVE = MoveGroupPythonIntefaceTutorial()
    print "==========================================================="
    print "=============    The NeuroSurgical System     ============="
    print "=============     Planning and Simulation     ============="
    print "==========================================================="

    print ""
    print "== Press `Enter` to get robot to optimal start position ==="
    raw_input()
    MOVE.go_to_joint_state()
    print "====          Robot in optimal start position          ===="
    
    print ""
    print ""
    print "==== Send Optimised Entry and Target Point from Slicer ===="
    print "====               Press Enter Once Sent               ===="
    MOVE.igtl_importer()
    raw_input()
    print ""

    print "==== Press `Enter` to execute movement to ENTRY point ====="
    raw_input()
    MOVE.go_to_pose_goal()

    print "==== Press `Enter` to execute movement to TARGET point ====="
    raw_input()
    plan, fraction = MOVE.plan_cartesian_path()
    MOVE.execute_plan(plan)

    print ""
    print "==========================================================="
    print "=============          Validate Pose          ============="
    print "==========================================================="
    print MOVE.group.get_current_pose()

    print ""
    print "=============      Same as target point       ============="
    print ""
    
    print ""
    print "==========================================================="
    print "=============      Planned and Executed!      ============="
    print "==========================================================="

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__': 
	main()




