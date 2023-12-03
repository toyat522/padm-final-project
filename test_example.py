from __future__ import print_function

import os
import sys
import argparse
import time
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, single_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, get_joint_positions, interval_generator, get_link_pose, interpolate_poses, set_renderer

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, ALL_JOINTS, compute_surface_aabb, rotate_robot, BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, add_ycb
from constants import *
from rrt import *

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

""" MAIN CODE """

def main():

    """ FIRST CHUNK OF CODE: WORLD SETUP """

    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=SUGAR_BOX_POSE)
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=SPAM_BOX_POSE)
    sugar_box_pose = get_pose(world.get_body(sugar_box))
    spam_box_pose = get_pose(world.get_body(spam_box))
    print("Sugar box pose:", sugar_box_pose)
    print("Spam box pose:", spam_box_pose)



    """ SECOND CHUNK OF CODE: JOINT SETUP """

    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    print('Kitchen Joints', [get_joint_name(world.kitchen, joint) for joint in world.kitchen_joints])
    print("All Bodies", world.all_bodies)
    print("Start pose:", start_pose)
    print("Joint positions:", get_joint_positions(world.robot, world.arm_joints)) # Tuple of 7 numbers

    wait_for_user()
    print("Open gripper")
    world.open_gripper()
    for _ in range(45):
        goal_pos = rotate_robot(world, -0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(MOVE_SLEEP)
    for _ in range(140):
        goal_pos = translate_linearly(world, 0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(MOVE_SLEEP)
    for _ in range(45):
        goal_pos = rotate_robot(world, 0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(MOVE_SLEEP)

    """ THIRD CHUNK OF CODE: MOVING THE ROBOT ARM """
#
#    wait_for_user()
    print("Open drawer")
    world.open_drawer()
#

#    print("Going to use IK to go from a sample start state to a goal state\n")
#    wait_for_user()
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)
#    print("Start pose:", start_pose)
#    #end_pose = (0.45, 1.2, -0.65), (-0.5, -0.5, 0.5, 0.5) # Drawer closed
#    end_pose = (0.75, 1.2, -0.65), (-0.5, -0.5, 0.5, 0.5) # Drawer opened
#    conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, end_pose, max_time=0.05), None)
#    if conf is None:
#        print('Failure!')
#    else:
#        print(conf)
#        set_joint_positions(world.robot, ik_joints, conf)

    wait_for_user()
    set_joint_positions(world.robot, ik_joints, STORE_SPAM_JOINT)
    set_pose(world.get_body("potted_meat_can1"), MOVED_SPAM_POSE)
    print("pose:", get_link_pose(world.robot, tool_link))

#
#
#    """ FOURTH CHUNK OF CODE: CHECKING DIFFERENT FUNCTIONS """
#
#
#    wait_for_user()
#    print("Close gripper")
#    world.close_gripper()
#
#    wait_for_user()
#    print("Open drawer")
#    world.open_drawer()
#
#
#
#    """ FIFTH CHUNK OF CODE: TRANSLATING THE ENTIRE ROBOT """
#
#    print("Going to operate the base WITH collision checking")
#    i = 0
#    while True:
#        goal_pos = translate_linearly(world, 0.01)
#        print("Goal pos:", goal_pos)
#        print("Robot position: ", get_joint_positions(world.robot, world.base_joints))
#        set_joint_positions(world.robot, world.base_joints, goal_pos)
#        time.sleep(MOVE_SLEEP) # Add for visual purposes
#        if (i % 30 == 0):
#            wait_for_user()
#        if pairwise_collision(world.robot, world.kitchen):
#            break
#        i += 1
#
#    """ SIXTH CHUNK OF CODE: MOVE ARM TO OBJECTS """
#    
#    print("Grab sugar box")
#    wait_for_user()
#    traj_gen = TrajectoryGenerator()
#    path = traj_gen.rrt(world, GRAB_SUGAR_JOINT)
#    for point in path:
#        print(pairwise_collision(world.robot, world.kitchen))
#        set_joint_positions(world.robot, ik_joints, point)
#        time.sleep(JOINT_MOVE_SLEEP)
#
#    print("Place sugar box")
#    wait_for_user()
#    path = traj_gen.rrt(world, PLACE_SUGAR_JOINT)
#    for point in path:
#        print(pairwise_collision(world.robot, world.kitchen))
#        set_joint_positions(world.robot, ik_joints, point)
#        time.sleep(JOINT_MOVE_SLEEP)
#
#    print("Grab spam box")
#    wait_for_user()
#    path = traj_gen.rrt(world, GRAB_SPAM_JOINT)
#    for point in path:
#        print(pairwise_collision(world.robot, world.kitchen))
#        set_joint_positions(world.robot, ik_joints, point)
#        time.sleep(JOINT_MOVE_SLEEP)
#
#    print("Move to closed drawer")
#    wait_for_user()
#    path = traj_gen.rrt(world, DRAWER_CLOSED_JOINT)
#    for point in path:
#        print(pairwise_collision(world.robot, world.kitchen))
#        set_joint_positions(world.robot, ik_joints, point)
#        time.sleep(JOINT_MOVE_SLEEP)
#
#    print("Move to open drawer")
#    wait_for_user()
#    path = traj_gen.rrt(world, DRAWER_OPEN_JOINT)
#    for point in path:
#        print(pairwise_collision(world.robot, world.kitchen))
#        set_joint_positions(world.robot, ik_joints, point)
#        time.sleep(JOINT_MOVE_SLEEP)
#
#    print("Open drawer")
#    world.open_drawer()
#
#    print("Place spam in drawer")
#    wait_for_user()
#    path = traj_gen.rrt(world, STORE_SPAM_JOINT)
#    for point in path:
#        print(pairwise_collision(world.robot, world.kitchen))
#        set_joint_positions(world.robot, ik_joints, point)
#        time.sleep(JOINT_MOVE_SLEEP)

    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
