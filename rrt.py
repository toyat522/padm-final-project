import os
import sys
import math

sys.setrecursionlimit(1500)
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import CIRCULAR_LIMITS, link_from_name, get_custom_limits, interval_generator, get_joint_positions, set_renderer, set_joint_positions, pairwise_collision
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints
from constants import *

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

class Tree:
    def __init__(self, point):
        self.parent = None
        self.children = []
        self.point = point
    
    def add_child(self, child):
        self.children.append(child)
        child.parent = self

class TrajectoryGenerator:

    def __init__(self, world):
        self.world = world
        self.tool_link = link_from_name(world.robot, 'panda_hand')
        self.ik_joints = get_ik_joints(world.robot, PANDA_INFO, self.tool_link)
        self.sample_free = get_sample_fn(world.robot, world.arm_joints)

    def solve(self, goal_point, *args):
        set_renderer(False)
        start_point = get_joint_positions(self.world.robot, self.world.arm_joints)
        tree = Tree(start_point)
        count = 1
        while True:
            rand_point = goal_point if count % GOAL_SAMPLE == 0 else self.sample_free() # Goal biasing
            nearest = self.find_nearest(tree, rand_point)
            new_point = self.steer(nearest.point, rand_point)
            if self.obstacle_free(new_point):
                last_node = Tree(new_point)
                nearest.add_child(last_node)
                if math.dist(new_point, goal_point) <= GOAL_THRESHOLD: break # In goal state
            count += 1
        set_joint_positions(self.world.robot, self.ik_joints, start_point)
        set_renderer(True)
        return self.find_path(last_node)

    def find_nearest(self, tree, point):
        if len(tree.children) == 0: return tree
        return min([tree] + [self.find_nearest(child, point) for child in tree.children], key=lambda x: math.dist(point, x.point))
        
    def steer(self, nearest_point, rand_point):
        distance = math.dist(nearest_point, rand_point)
        if distance <= JOINT_STEP_SIZE: return rand_point
        return tuple((rand_point[i] - nearest_point[i]) * (JOINT_STEP_SIZE / distance) + nearest_point[i] for i in range(7))

    def obstacle_free(self, point):
        set_joint_positions(self.world.robot, self.ik_joints, point)
        return not pairwise_collision(self.world.robot, self.world.kitchen)

    def find_path(self, goal):
        curr = goal
        path = [curr.point]
        while curr.parent:
            curr = curr.parent
            path.append(curr.point)
        path.reverse()
        return path
