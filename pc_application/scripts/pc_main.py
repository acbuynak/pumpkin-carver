#!/usr/bin/env python3
#
# Software License Agreement (Apache 2.0 License)
# Copyright (c) 2022, A.C. Buynak
# 
# Author: A.C. Buynak

# Description:
# System control for a pumpking carving robot!
#

# Utilities
from turtle import position
import numpy as np

# ROS
import rospy
from arp_msgs.srv import GenerateMotionPlan, GenerateMotionPlanRequest
from arp_msgs.srv import ExecuteMotionPlan, ExecuteMotionPlanRequest
from pc_canvas_locking.srv import WorkpieceLocking, WorkpieceLockingRequest

from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from geometry_msgs.msg import Point, Quaternion
from arp_msgs.msg import ToolPath, ToolPaths



#####################
# Control Variables #
#####################

WKPC_FRAME = "canvas"


#####################
# Support Functions #
#####################

def setCanvasOrigin():
    """
    @brief Call workpiece locking service with specified frames to define the
    position of the pumpkin in the scene

    @return bool
    """
    canvas_client = rospy.ServiceProxy('/locker', WorkpieceLocking)

    req = WorkpieceLockingRequest()
    req.locator_tcp_frame = "mill_tcp"
    req.origin_frame = "r3_base_link"
    req.workpiece_frame = "canvas"

    result = canvas_client(req)

    return result.success



########################
# Tool Path Generators #
########################

def pathBlockO(height_inch: float, carve_depth_inch: float) -> PoseArray:
    """
    @brief Develops a path plan to carve a BLOCK 'O' of size.
    Size is defined by height in inches. 
    Canvas origin should be the center bottom of the pumpkin carving area.
    Z-axis points into the pumpkin.

    @param scale = Height of BlockO
    @param carve_depth_inch = Total carve depth
    @return geometry_msgs::PoseArray
    """

    DEPTH = carve_depth_inch * 0.02540000

    # Block O Definition (1-inch tall)(convert from inch to meters)
    half_block = np.array([[0.0, 0.05, 0.0],
                           [0.23, 0.05, 0],
                           [0.23, 0.11, 0],
                           [0.29, 0.17, 0],
                           [0.71, 0.17, 0],
                           [0.77, 0.11, 0],
                           [0.77, 0.05, 0],
                           [1.0, 0.05, 0],
                           [1.0, 0.21, 0],
                           [0.8, 0.41, 0],
                           [0.2, 0.41, 0],
                           [0, 0.21, 0],
                           [0, 0.05, 0]]
    )
    half_block = half_block * height_inch * 0.02540000

    # Setup Tool Path Plan
    tp = ToolPath()
    tp.use_tool = False
    tp.velocity = 0.05 # m/s


    ### Segment 0 ###
    # PreCarving Standoff
    # standoff from origin, move sidesways, then straight into pumpkin
    seg = PoseArray()
    seg.header.frame_id = WKPC_FRAME
    seg.header.seq = 0

    p = Pose()
    p.position = Point(0, 0, -0.0254)
    seg.poses.append(p)

    p.position = Point(half_block[0][0], half_block[0][1], -0.0254)
    seg.poses.append(p)

    tp.segments.append(seg)
    tp.pilz_algorithm.append("PTP")
    del seg, p


    ### Segment 1 ###
    # Right Half of Block O
    seg = PoseArray()
    seg.header.frame_id = WKPC_FRAME
    seg.header.seq = 1

    p = Pose()
    p.orientation = Quaternion(0, 0, 0, 1)
    for pt in half_block:
        print(pt)
        p.position = Point(pt[0], pt[1], pt[2]+DEPTH)
        seg.poses.append(p)
    
    p.position = Point(half_block[-1][0], half_block[-1][1], -0.0254)
    seg.poses.append(p)

    tp.segments.append(seg)
    tp.pilz_algorithm.append("LIN")
    del seg, p, pt


    ### Segment 2 ###
    # Left Half of Block O
    seg = PoseArray()
    seg.header.frame_id = WKPC_FRAME
    seg.header.seq = 1

    p = Pose()
    p.orientation = Quaternion(0, 0, 0, 1)
    for pt in half_block:
        print(pt)
        p.position = Point(pt[0], -pt[1], pt[2]+DEPTH)
        seg.poses.append(p)
    
    p.position = Point(half_block[-1][0], -half_block[-1][1], -0.0254)
    seg.poses.append(p)

    tp.segments.append(seg)
    tp.pilz_algorithm.append("LIN")
    del seg, p, pt

    return tp


def buildToolPathPlanRequest() -> GenerateMotionPlanRequest:
    """
    @brief Assembles a ROS Service request structured message to send to the planning_server.

    @return arp_msgs::GenerateMotionPlan::Request
    """

    req = GenerateMotionPlanRequest()

    req.motion_group = "mh50_mill_eef"
    req.tcp_frame = "mill_tcp"

    # TODO: Update path parameters
    req.tool_paths = pathBlockO(height_inch=6, carve_depth_inch=0)

    return req



##########
## MAIN ##
##########

def main():

    # Wait for Service
    rospy.logwarn("Waiting for workpiece locking, traj generation, and traj execution servers to come online...")
    rospy.wait_for_service('/locker')
    rospy.wait_for_service('/arp_planning_server/motion_planner')
    rospy.wait_for_service('/arp_execution_server/execute_motion_plan')

    # Get Start Canvas Origin
    if not setCanvasOrigin():
        rospy.logfatal("Unable to set canvas origin.")

    # Build Test Request
    test_request = buildToolPathPlanRequest()

    # Motion Testing
    try:

        # Call Trajectory Generation
        rospy.logwarn("Attempting to call Motion Generation service...")

        generation_server = rospy.ServiceProxy('/arp_planning_server/motion_planner', GenerateMotionPlan)
        traj = generation_server(test_request)
        print(traj)

        # User Check
        input("Are you ready to move the robot? (y = any key | CTRL-D = Stop)")

        # Call Trajectory Execution
        rospy.logwarn("Attempting to call Motion Execution service...")
        execution_server = rospy.ServiceProxy('/arp_execution_server/execute_motion_plan', ExecuteMotionPlan)

        mp = ExecuteMotionPlanRequest()
        mp.tool_motion_plans = traj.tool_motion_plans
        resp = execution_server(mp)
        print(resp)


    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


    rospy.logwarn("END TEST")


if __name__ == "__main__":
    pathBlockO(height_inch=6, carve_depth_inch=0)

    # main()