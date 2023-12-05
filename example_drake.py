from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import matplotlib.pyplot as plt

prog = MathematicalProgram()
p1 = prog.NewContinuousVariables(2, "p1")
p2 = prog.NewContinuousVariables(2, "p2")

# Add the constraint that p1 is on the unit circle centered at (0, 2)
prog.AddConstraint(
    lambda z: [z[0]**2 + (z[1]-2)**2],
    lb=np.array([1.]),
    ub=np.array([1.]),
    vars=p1)

# Add the constraint that p2 is on the curve y=x*x
prog.AddConstraint(
    lambda z: [z[1] - z[0]**2],
    lb=[0.],
    ub=[0.],
    vars=p2)

# Add the cost on the distance between p1 and p2
prog.AddQuadraticCost((p1-p2).dot(p1-p2))

# Set the value of p1 in initial guess to be [0, 1]
prog.SetInitialGuess(p1, [0., 1.])
# Set the value of p2 in initial guess to be [1, 1]
prog.SetInitialGuess(p2, [1., 1.])

# Now solve the program
result = Solve(prog)
print(f"Is optimization successful? {result.is_success()}")
p1_sol = result.GetSolution(p1)
p2_sol = result.GetSolution(p2)
print(f"solution to p1 {p1_sol}")
print(f"solution to p2 {p2_sol}")
print(f"optimal cost {result.get_optimal_cost()}")

# Plot the solution.
plt.figure()
plt.plot(np.cos(np.linspace(0, 2*np.pi, 100)), 2+np.sin(np.linspace(0, 2*np.pi, 100)))
plt.plot(np.linspace(-2, 2, 100), np.power(np.linspace(-2, 2, 100), 2))
plt.plot(p1_sol[0], p1_sol[1], '*')
plt.plot(p2_sol[0], p2_sol[1], '*')
plt.axis('equal')
plt.show()
