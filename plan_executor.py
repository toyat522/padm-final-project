from __future__ import print_function

import os
import sys

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import wait_for_user

from src.world import World
from src.utils import add_ycb
from robot import Robot
from ff_planner import Planner
from constants import *

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def main():

    # Planning
    domain = "pddl/domain.pddl"
    problem = "pddl/problem.pddl"
    planner = Planner()
    plan = planner.solve_ff(domain, problem)

    # Stop if no plan found
    if plan is None:
        print("No plan was found!")
        return

    # Setup world
    world = World(use_gui=True)
    add_sugar_box(world, idx=0, counter=1, pose2d=SUGAR_BOX_POSE)
    add_spam_box(world, idx=1, counter=0, pose2d=SPAM_BOX_POSE)
    world._update_initial()

    # Execute plan
    wait_for_user()
    robot = Robot(world)
    robot.move_base()
    for act in plan:
        print(act)
        robot.act(act.name, act.parameters[0])
    robot.move_joint_init()

    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
