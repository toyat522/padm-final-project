from __future__ import print_function

import os
import sys
import argparse
import time
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, get_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, compute_surface_aabb, rotate_robot, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, \
    add_ycb, get_sample_fn


""" CONSTANT DECLARATION """

SUGAR_BOX_POSE  = (-0.2, 0.65, np.pi / 4) # Starting position of sugar box
SPAM_BOX_POSE   = (0.2, 1.1, np.pi / 4)   # Starting position of spam box
MOVE_SLEEP      = 0.03                    # Sleep time for translation (for animation purposes)
POS_STEP_SIZE   = 0.01                    # Step size of robot arm interpolation and RRT

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
    wait_for_user()



    """ SECOND CHUNK OF CODE: JOINT SETUP """

    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    print('Kitchen Joints', [get_joint_name(world.kitchen, joint) for joint in world.kitchen_joints])
    print("All Bodies", world.all_bodies)
    sample_fn = get_sample_fn(world.robot, world.arm_joints)



    """ THIRD CHUNK OF CODE: MOVING THE ROBOT ARM """

    # TODO: HOW TO CHECK FOR COLLISION WITHOUT "ACTUALLY" MOVING THE ARM?

#    print("Going to use IK to go from a sample start state to a goal state\n")
#    for i in range(2):
#        print('Iteration:', i)
#        conf = sample_fn() # Starting randomly sampled configuration
#        set_joint_positions(world.robot, world.arm_joints, conf)
#        print("Joint positions:", get_joint_positions(world.robot, world.arm_joints)) # Tuple of 7 numbers
#        wait_for_user()
#        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
#        start_pose = get_link_pose(world.robot, tool_link)
#        print("Start pose:", start_pose)
#        end_pose = multiply(start_pose, Pose(Point(z=1.0)))
#
#        # Interpolate poses from start to end
#        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
#
#            # Calculate the closest inverse kinematics. Try calculating for max_time until it gives up.
#            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
#
#            # If inverse kinematics failed, then print "Failure!"
#            if conf is None:
#                print('Failure!')
#                wait_for_user()
#                break
#
#            # Move the joints
#            set_joint_positions(world.robot, ik_joints, conf)
#
#
#    """ FOURTH CHUNK OF CODE: CHECKING DIFFERENT FUNCTIONS """
#
#    wait_for_user()
#    print("Open gripper")
#    world.open_gripper()
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

    for _ in range(50):
        goal_pos = rotate_robot(world, -0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(MOVE_SLEEP)
    for _ in range(135):
        goal_pos = translate_linearly(world, 0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(MOVE_SLEEP)
    for _ in range(50):
        goal_pos = rotate_robot(world, 0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(MOVE_SLEEP)

    """ SIXTH CHUNK OF CODE: MOVE ARM TO OBJECTS """
    
    wait_for_user()
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)
    print("Start pose:", start_pose)

    # Move to spam box
    for pose in interpolate_poses(start_pose, spam_box_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        if conf is None:
            print('Failure!')
            wait_for_user()
            break
        set_joint_positions(world.robot, ik_joints, conf)

    # Move to sugar box
    for pose in interpolate_poses(start_pose, sugar_box_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        if conf is None:
            print('Failure!')
            wait_for_user()
            break
        set_joint_positions(world.robot, ik_joints, conf)

    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
