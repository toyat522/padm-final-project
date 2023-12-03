import time
from pybullet_tools.utils import set_joint_positions
from src.utils import rotate_robot, translate_linearly, add_ycb
from constants import *
from rrt import *

class Robot:

    def move_base(self, world):
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
        world.open_gripper()

    def grab_sugar(self, world, ik_joints):
        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, GRAB_SUGAR_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.close_gripper()

    def place_sugar(self, world, ik_joints):
        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, PLACE_SUGAR_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.open_gripper()

    def grab_spam(self, world, ik_joints):
        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, GRAB_SPAM_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.close_gripper()

    def place_spam(self, world, ik_joints):
        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, STORE_SPAM_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.open_gripper()

    def open_drawer(self, world, ik_joints):
        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, DRAWER_CLOSED_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.close_gripper()

        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, DRAWER_OPEN_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.open_gripper()
        world.open_drawer()

    def close_drawer(self, world, ik_joints):
        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, DRAWER_OPEN_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.close_gripper()
        world.close_drawer()

        time.sleep(ACTION_SLEEP)
        path = TrajectoryGenerator().rrt(world, DRAWER_CLOSED_JOINT)
        for point in path:
            set_joint_positions(world.robot, ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        world.open_gripper()
