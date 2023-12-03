from __future__ import print_function

import os
import sys
import time
import numpy as np

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import wait_for_user, link_from_name

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints

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
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    add_sugar_box(world, idx=0, counter=1, pose2d=SUGAR_BOX_POSE)
    add_spam_box(world, idx=1, counter=0, pose2d=SPAM_BOX_POSE)
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)

    # Initial movement
    robot = Robot()
    robot.move_base(world)
    function_map = {
        "open_storage":     {"drawer": robot.open_drawer},
        "close_storage":    {"drawer": robot.close_drawer},
        "pick_up_static":   {"sugar": robot.grab_sugar,  "spam": robot.grab_spam},
        "pick_up_openable": {"sugar": robot.grab_sugar,  "spam": robot.grab_spam},
        "place_openable":   {"sugar": robot.place_sugar, "spam": robot.place_spam},
        "place_static":     {"sugar": robot.place_sugar, "spam": robot.place_spam},
    }

    # Execute plan
    for act in plan:
        act_name  = act.name
        act_param = act.parameters[0]
        function_map[act_name][act_param](world, ik_joints)

    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()
