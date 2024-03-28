import numpy as np
from scipy.optimize import minimize


class NonlinearMPCController:
    def __init__(self, dynamics_func, cost_func, u_bounds, x_bounds, N):
        self.dynamics_func = dynamics_func  # Nonlinear dynamics function f(x, u)
        self.cost_func = cost_func  # Cost function J(x, u)
        self.u_bounds = u_bounds  # Control input bounds (u_min, u_max)
        self.x_bounds = x_bounds  # State bounds (x_min, x_max)
        self.N = N  # Prediction horizon

    def predict_control(self, x0):
        # Define optimization variables for control inputs over the horizon
        U = np.zeros((self.N, len(x0)))  # N control actions of the same size as state

        # Define cost and constraints for optimization
        def objective(U_flat):
            U = U_flat.reshape((self.N, len(x0)))
            cost = 0
            x = x0
            for u in U:
                cost += self.cost_func(x, u)
                x = self.dynamics_func(x, u)
            return cost

        def constraint(U_flat):
            U = U_flat.reshape((self.N, len(x0)))
            x = x0
            for u in U:
                x = self.dynamics_func(x, u)
            # Final state should be within bounds
            return np.concatenate((x - self.x_bounds[0], self.x_bounds[1] - x))

        # Bounds for each control input over the horizon
        bounds = [self.u_bounds for _ in range(self.N * len(x0))]

        # Flatten U for the optimizer
        U_flat = U.flatten()

        # Nonlinear constraint
        nonlinear_constraint = {'type': 'ineq', 'fun': constraint}

        # Solve the optimization problem
        result = minimize(objective, U_flat, bounds=bounds, constraints=[nonlinear_constraint])

        if result.success:
            U_optimal = result.x.reshape((self.N, len(x0)))
            return U_optimal[0]  # Return the first control action
        else:
            raise ValueError("Optimization failed")


# Define your system's nonlinear dynamics
def dynamics(x, u):
    # Insert your system's dynamics here
    # For example, a simple nonlinear system could be:
    return x + u - 0.1 * x ** 2


# Define your cost function
def cost(x, u):
    # Insert your cost function here
    # For example:
    Q = np.diag([1, 1])
    R = np.diag([1, 1])
    return x @ Q @ x + u @ R @ u


# Define control input bounds and state bounds
u_bounds = (-1, 1)
x_bounds = (-10, 10)

# Prediction horizon
N = 10

# Create an instance of the controller
nmpc = NonlinearMPCController(dynamics, cost, u_bounds, x_bounds, N)

# Simulate the system
x0 = np.array([2, -2])  # Initial state
x_history = [x0]
u_history = []

for _ in range(20):
    u = nmpc.predict_control(x_history[-1])
    x_next = dynamics(x_history[-1], u)
    x_history.append(x_next)
    u_history.append(u)

# Convert history to numpy arrays for plotting
x_history = np.array(x_history)
u_history = np.array(u_history)

# Plotting the results
import matplotlib.pyplot as plt

plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(x_history[:, 0], label='State x1')
plt.plot(x_history[:, 1], label='State x2')
plt.title('State Trajectories')
plt.legend()

plt.subplot(1, 2, 2)
plt
