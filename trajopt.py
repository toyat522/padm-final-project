import numpy as np
import os
import sys

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pydrake.solvers import MathematicalProgram, Solve, Binding
from pybullet_tools.utils import CIRCULAR_LIMITS, link_from_name, get_custom_limits, interval_generator, get_joint_positions, set_renderer, set_joint_positions, pairwise_collision, get_link_pose
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO
from pybullet_tools.ikfast.ikfast import get_ik_joints
from constants import *

class TrajectoryOptimizer:

    def __init__(self, world):
        self.world = world
        self.tool_link = link_from_name(world.robot, 'panda_hand')
        self.ik_joints = get_ik_joints(world.robot, PANDA_INFO, self.tool_link)

    def solve(self, goal_point):
        set_renderer(False)
        start_point = np.array(get_joint_positions(self.world.robot, self.world.arm_joints))

        # Define the number of joints and the number of time steps
        num_joints = 7

        # Create a MathematicalProgram
        prog = MathematicalProgram()

        # Create decision variables for joint positions, velocities, and time
        q_vars = prog.NewContinuousVariables(PATH_LENGTH, num_joints, "q")
        time_var = prog.NewContinuousVariables(1, "time")[0]

        # Set the linspace as the initial guess
        prog.SetInitialGuess(q_vars, np.linspace(start_point, goal_point, PATH_LENGTH))

        # Set initial and final joint positions
        q0 = start_point                    # Initial joint positions
        qf = np.array(goal_point)  # Final joint positions

        # Add constraints to ensure the initial and final joint positions and velocities
        for i in range(num_joints):
            prog.AddConstraint(q_vars[0, :][i] == q0[i])
            prog.AddConstraint(q_vars[-1, :][i] == qf[i])

        # Add a cost to minimize time
        prog.AddLinearCost(time_var)
        prog.AddLinearConstraint(time_var >= 1)

        # Add constraints to ensure the arm doesn't teleport
        for i in range(PATH_LENGTH - 1):
            for j in range(num_joints):
                prog.AddConstraint((q_vars[i + 1, :][j] - q_vars[i, :][j]) / (time_var / PATH_LENGTH) <= 1)
                prog.AddConstraint((q_vars[i + 1, :][j] - q_vars[i, :][j]) / (time_var / PATH_LENGTH) >= -1)

        # Add joint constraints
        lower, upper = get_custom_limits(self.world.robot, self.world.arm_joints, circular_limits=CIRCULAR_LIMITS)
        for i in range(PATH_LENGTH):
            for j in range(num_joints):
                prog.AddConstraint(lower[j] <= q_vars[i, :][j])
                prog.AddConstraint(q_vars[i, :][j] <= upper[j])

#        # Add collision avoidance constraint
#        for i in range(PATH_LENGTH):
#            prog.AddConstraint(self.obstacle_free, 1, 1, q_vars[i, :])

        # Solve the optimization problem
        result = Solve(prog)

        # Extract the optimized joint positions, velocities, and time
        q_opt = result.GetSolution(q_vars)
        set_renderer(True)

        return q_opt

    def obstacle_free(self, q):
        point = []
        for val in q:
            point.append(val.value())
        set_joint_positions(self.world.robot, self.ik_joints, point)
        return not pairwise_collision(self.world.robot, self.world.kitchen)

#    def fk(self, q):
#        set_joint_positions(self.world.robot, self.ik_joints, q)
#        return [get_link_pose(self.world.robot, self.tool_link)]

