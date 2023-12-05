import time
from pybullet_tools.utils import set_joint_positions, set_pose, get_link_pose, link_from_name
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints
from src.utils import rotate_robot, translate_linearly
from constants import *
from rrt import TrajectoryGenerator
from trajopt import TrajectoryOptimizer

class Robot:

    def __init__(self, world, use_trajopt=False):
        self.world = world
        self.tool_link = link_from_name(world.robot, "panda_hand")
        self.ik_joints = get_ik_joints(world.robot, PANDA_INFO, self.tool_link)
        self.sugar_box = self.world.get_body("sugar_box0")
        self.spam_box = self.world.get_body("potted_meat_can1")
        if use_trajopt:
            self.trajgen = TrajectoryOptimizer(world)
        else:
            self.trajgen = TrajectoryGenerator(world)

        self.function_map = {
            "open_storage":     {"drawer": self.open_drawer},
            "close_storage":    {"drawer": self.close_drawer},
            "pick_up_static":   {"sugar":  self.grab_sugar,  "spam": self.grab_spam},
            "pick_up_openable": {"sugar":  self.grab_sugar,  "spam": self.grab_spam},
            "place_openable":   {"sugar":  self.place_sugar, "spam": self.place_spam},
            "place_static":     {"sugar":  self.place_sugar, "spam": self.place_spam},
        }

    def move_base(self):
        for _ in range(45):
            goal_pos = rotate_robot(self.world, -0.01)
            set_joint_positions(self.world.robot, self.world.base_joints, goal_pos)
            time.sleep(MOVE_SLEEP)
        for _ in range(140):
            goal_pos = translate_linearly(self.world, 0.01)
            set_joint_positions(self.world.robot, self.world.base_joints, goal_pos)
            time.sleep(MOVE_SLEEP)
        for _ in range(45):
            goal_pos = rotate_robot(self.world, 0.01)
            set_joint_positions(self.world.robot, self.world.base_joints, goal_pos)
            time.sleep(MOVE_SLEEP)
        self.world.open_gripper()

    def move_joint_init(self):
        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(INIT_JOINT)
        for point in path:
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)

    def grab_sugar(self):
        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(GRAB_SUGAR_JOINT)
        for point in path:
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        self.world.close_gripper()
        set_pose(self.sugar_box, get_link_pose(self.world.robot, self.tool_link))

    def place_sugar(self):
        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(PLACE_SUGAR_JOINT)
        for point in path:
            set_pose(self.sugar_box, get_link_pose(self.world.robot, self.tool_link))
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        self.world.open_gripper()
        set_pose(self.sugar_box, MOVED_SUGAR_POSE)

    def grab_spam(self):
        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(GRAB_SPAM_JOINT)
        for point in path:
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        self.world.close_gripper()
        set_pose(self.spam_box, get_link_pose(self.world.robot, self.tool_link))

    def place_spam(self):
        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(STORE_SPAM_JOINT)
        for point in path:
            set_pose(self.spam_box, get_link_pose(self.world.robot, self.tool_link))
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        self.world.open_gripper()
        set_pose(self.spam_box, MOVED_SPAM_POSE)

    def open_drawer(self):
        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(DRAWER_CLOSED_JOINT)
        for point in path:
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        self.world.close_gripper()

        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(DRAWER_OPEN_JOINT)
        for point in path:
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        self.world.open_gripper()
        self.world.open_drawer()

    def close_drawer(self):
        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(DRAWER_OPEN_JOINT)
        for point in path:
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        set_pose(self.spam_box, STORED_SPAM_POSE)
        self.world.close_gripper()
        self.world.close_drawer()

        time.sleep(ACTION_SLEEP)
        path = self.trajgen.solve(DRAWER_CLOSED_JOINT)
        for point in path:
            set_joint_positions(self.world.robot, self.ik_joints, point)
            time.sleep(JOINT_MOVE_SLEEP)
        self.world.open_gripper()

    def act(self, act_name, act_param):
        self.function_map[act_name][act_param]()
