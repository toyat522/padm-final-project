import numpy as np
import os
import sys

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits
from pydrake.all import MathematicalProgram, Solve
from constants import *

# Define the number of joints and the number of time steps
num_joints = 7
num_time_steps = 200  # You can adjust this based on your requirements

# Create a MathematicalProgram
prog = MathematicalProgram()

# Create decision variables for joint positions, velocities, and time
q_vars = prog.NewContinuousVariables(num_time_steps, num_joints, "q")
time_var = prog.NewContinuousVariables(1, "time")[0]

# Set initial and final joint positions
q0 = np.array(INIT_JOINT)  # Initial joint positions
qf = np.array(GRAB_SPAM_JOINT)  # Final joint positions

# Set initial and final joint velocities
qd0 = np.zeros(num_joints)
qdf = np.zeros(num_joints)

# Add constraints to ensure the initial and final joint positions and velocities
for i in range(num_joints):
    prog.AddConstraint(q_vars[0, :][i] == q0[i])
    prog.AddConstraint(q_vars[-1, :][i] == qf[i])

# Add a cost to minimize time
prog.AddLinearCost(time_var)
prog.AddLinearConstraint(time_var >= 1)

# Add constraints to ensure the arm doesn't teleport
for i in range(num_time_steps - 1):
    for j in range(num_joints):
        prog.AddConstraint((q_vars[i + 1, :][j] - q_vars[i, :][j]) / (time_var / num_time_steps) <= 0.01)
        prog.AddConstraint((q_vars[i + 1, :][j] - q_vars[i, :][j]) / (time_var / num_time_steps) >= -0.01)

lower = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
upper = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)

# Add joint constraints
for i in range(num_time_steps):
    for j in range(num_joints):
        prog.AddConstraint(lower[j] <= q_vars[i, :][j])
        prog.AddConstraint(q_vars[i, :][j] <= upper[j])

# Solve the optimization problem
result = Solve(prog)

# Extract the optimized joint positions, velocities, and time
q_opt = result.GetSolution(q_vars)
time_opt = result.GetSolution(time_var)

# Print or visualize the results as needed
print("Optimized Joint Positions:", q_opt)
print("Optimized Time:", time_opt)
