from __future__ import print_function

import os
import sys
import argparse
import time
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly


""" CONSTANT DECLARATION """

UNIT_POSE2D     = (0., 0., 0.)
SUGAR_BOX_POSE  = (-0.2, 0.65, np.pi / 4) # Starting position of sugar box
SPAM_BOX_POSE   = (0.2, 1.1, np.pi / 4)   # Starting position of spam box
TRANSLATE_SLEEP = 0.03                    # Sleep time for translation (for animation purposes)
POS_STEP_SIZE   = 0.01                    # Step size of robot arm interpolation and RRT



""" GIVEN HELPER FUNCTIONS: DO NOT EDIT! """

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn



""" MAIN CODE """

def main():

    """ FIRST CHUNK OF CODE: WORLD SETUP """

    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=SUGAR_BOX_POSE)
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=SPAM_BOX_POSE)
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

#    print("Going to use IK to go from a sample start state to a goal state\n")
#    for i in range(2):
#        print('Iteration:', i)
#        conf = sample_fn() # Starting randomly sampled configuration
#        set_joint_positions(world.robot, world.arm_joints, conf)
#        wait_for_user()
#        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
#        start_pose = get_link_pose(world.robot, tool_link)
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


    """ FOURTH CHUNK OF CODE: CHECKING DIFFERENT FUNCTIONS """

    wait_for_user()
    print("Open gripper")
    world.open_gripper()

    wait_for_user()
    print("Close gripper")
    world.close_gripper()

    wait_for_user()
    print("Open drawer")
    world.open_drawer()


    """ FIFTH CHUNK OF CODE: TRANSLATING THE ENTIRE ROBOT """

    print("Going to operate the base without collision checking")
    i = 0
    while True:
        goal_pos = translate_linearly(world, 0.01)
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(TRANSLATE_SLEEP) # Add for visual purposes
        if (i % 30 == 0):
            wait_for_user()
        i += 1
    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
