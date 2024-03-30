import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import time


class SimpleMPC:
    def __init__(self, A, B, Q, R, F, N):
        self.A = A              # State transition matrix
        self.B = B              # Control matrix
        self.Q = Q              # State cost matrix
        self.R = R              # Control cost matrix
        self.F = F              # Terminal state cost matrix
        self.N = N              # Prediction horizon
        self.n = A.shape[0]     # Number of states
        self.p = B.shape[0]     # Number of control inputs
        self.E, self.H = self._compute_matrices()
        self.X_K = np.zeros((self.N, self.n))

    def _compute_matrices(self):
        A = self.A
        B = self.B
        Q = self.Q
        R = self.R
        F = self.F
        N = self.N
        n = self.n
        p = self.p

        M = np.vstack((np.eye(n), np.zeros((N * n, n))))  # (N+1)*n x n 的矩阵，上部分为单位矩阵，下部分先为0矩阵
        C = np.zeros(((N + 1) * n, N * p))                # (N+1)*n x N*p 的矩阵
        tmp = np.eye(n)                                   # n x n 的单位矩阵
        # 打印所有的矩阵
        # print("A", A, "B", B, "Q", Q, "R", R, "F", F, "N", N)
        # print("n:", n, "p:", p)

        for i in range(0, N):
            rows = slice((i+1)*n, (i+2)*n)              # 切片 （i+1）*n 到 (i+2)*n 行

            C[rows, :] = np.hstack((tmp @ B, C[rows.start - n:rows.stop - n, :-p]))
            tmp = A @ tmp
            M[rows, :] = tmp
            # print("rows", rows)
            # print("C", C)
            # print("M", M)

        Q_bar = np.kron(np.eye(N), Q)
        Q_bar = np.block([
            [Q_bar, np.zeros((Q_bar.shape[0], F.shape[1]))],  # Q_bar和右侧的零矩阵
            [np.zeros((F.shape[0], Q_bar.shape[1])), F] ])   # 下方的零矩阵和F
        R_bar = np.kron(np.eye(N), R)
        # print("Q_bar", Q_bar)
        # print("R_bar", R_bar)

        G = M.T @ Q_bar @ M             # n x n 的矩阵
        E = C.T @ Q_bar @ M             # N*p x n 的矩阵
        H = C.T @ Q_bar @ C + R_bar     # N*p x N*p 的矩阵

        # print("G", G)
        # print("E", E)
        # print("H", H)
        return E, H

    def _cost(self, u, x_k):
        """
        计算预测空间代价函数总和
        :param u:   连续N个控制输入向量组成
        :param x_k: k时刻的状态（预测空间初始状态
        :return: 代价函数值
        """
        u = np.reshape(u, (-1, 1))  # Ensure U is a column vector
        return 0.5 * u.T @ self.H @ u + (self.E @ x_k).T @ u

    def predict_control(self, x_k):
        """
        Predict the optimal control input at time k
        :param x_k:
        :return:
        """
        x_k = np.reshape(x_k, (-1, 1))  # Ensure x_k is a column vector
        # options = {'maxiter': 100}  # 限制最大迭代次数
        result = minimize(self._cost, np.zeros(self.N * self.p), args=(x_k,), method='SLSQP')
        if result.success:
            u = np.reshape(result.x, (-1, 1))        # result.x为最优解
            return u[:self.p].flatten()
        else:
            raise ValueError("Optimization failed")


if __name__ == '__main__':
    ex_A = np.array([[1, 0.1],
                     [-1, 2]])
    ex_B = np.array([[0.2, 1],
                     [0.5, 2]])
    ex_Q = np.array([[100, 0],
                     [0, 1]])
    ex_F = np.array([[100, 0],
                     [0, 1]])
    ex_R = np.array([[1, 0],
                     [0, 0.1]])
    k_steps = 100                           # 仿真运行步数
    U_k_steps = np.zeros((2, k_steps))      # 每个时间步计算出的最优控制输入 p x k_steps
    X_k_steps = np.zeros((2, k_steps))      # 每个时间步的状态   n x k_steps
    X_k_steps[:, 0] = np.array([20, -20])      # 初始状态
    test_mpc = SimpleMPC(ex_A, ex_B, ex_Q, ex_R, ex_F, 8)
    delta_time_sum = 0
    for i in range(k_steps-1):
        start_time = time.time()
        U_k_steps[:, i] = test_mpc.predict_control(X_k_steps[:, i])
        X_k_steps[:, i + 1] = ex_A @ X_k_steps[:, i] + ex_B @ U_k_steps[:, i]
        delta_time_sum += time.time() - start_time
        # print(f"Time taken for iteration {i}: {delta_time:.6f}s")
    print(f"Average time taken per iteration: {delta_time_sum / k_steps:.6f}s")

    # 绘图
    plt.subplot(2, 2, 1)
    plt.plot(X_k_steps[0, :], label='State 1')
    plt.title('State 1 over time')
    plt.xlabel('Time Step')
    plt.ylabel('State 1 Value')
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(X_k_steps[1, :], label='State 2')
    plt.title('State 2 over time')
    plt.xlabel('Time Step')
    plt.ylabel('State 2 Value')
    plt.legend()

    # 控制输入图
    plt.subplot(2, 2, 3)
    plt.plot(U_k_steps[0, :], label='Control Input 1')
    plt.title('Control Input 1 over time')
    plt.xlabel('Time Step')
    plt.ylabel('Control Input 1 Value')
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(U_k_steps[1, :], label='Control Input 2')
    plt.title('Control Input 2 over time')
    plt.xlabel('Time Step')
    plt.ylabel('Control Input 2 Value')
    plt.legend()

    plt.tight_layout()
    plt.show()

class NonlinearMPCController:
    def __init__(self, dynamics_func, cost_func, horizon, n, p, x_bounds, u_bounds):
        self.dynamics_func = dynamics_func  # Nonlinear dynamics function f(x, u)
        self.cost_func = cost_func          # Cost function J(x, u)
        self.N = horizon                    # Prediction horizon
        self.n = n                          # Number of states
        self.p = p                          # Number of control outputs
        self.x_bounds = x_bounds            # State bounds nx2: lower bound in 1st column, upper bound in 2nd column
        self.u_bounds = u_bounds            # Control bounds px2

    def predict_control(self, x_init):
        """
        Predict the optimal control output at time k
        :param x_init: Initial state at time k
        :return:
        """

        def objective(u_list_flat):
            """
            计算预测空间里代价和
            :param u_list_flat: 一维化的控制输入序列
            :return: cost_sum: 代价和
            """
            u_list = u_list_flat.reshape((self.N, self.p))    # Reshape U from 1xNp to be NxP
            cost_sum = 0
            x = x_init
            for u in u_list:        # 按行遍历，需要转置
                u = u.reshape((self.p, 1))
                cost_sum += self.cost_func(x, u)
                x = self.dynamics_func(x, u)
            return cost_sum

        def constraint(u_list_flat):
            u_list = u_list_flat.reshape((self.N, self.p))
            x = x_init
            for u in u_list:
                u = u.reshape((self.p, 1))
                x = self.dynamics_func(x, u)
            # 只对最终状态进行了约束, 越界的将返回负数
            lower_bounds = np.array([b[0] for b in self.x_bounds]).reshape(-1, 1)
            upper_bounds = np.array([b[1] for b in self.x_bounds]).reshape(-1, 1)
            return np.concatenate(((x - lower_bounds).flatten(), (upper_bounds - x).flatten()))

        u_list_flat0 = np.zeros(self.p * self.N)  # Initial guess for control actions
        # Nonlinear constraint
        nonlinear_constraint = {'type': 'ineq',
                                'fun': constraint}
        # Solve the optimization problem
        flat_u_bounds = [(bound[0], bound[1]) for bound in self.u_bounds for _ in range(self.N)]
        result = minimize(objective, u_list_flat0, bounds=flat_u_bounds, constraints=[nonlinear_constraint])

        if result.success:
            u_list_opti = result.x.reshape((self.N, self.p))
            return u_list_opti[0].reshape((self.p, 1))              # Return the first control action
        else:
            raise ValueError("Optimization failed")


# Define your system's nonlinear dynamics
def dynamics(x_vec, u_vec):
    """
    状态转移函数
    :param x_vec:  当前状态向量
    :param u_vec:  控制输入
    :return: next_x_vec 下一个状态
    """
    next_x_vec = np.zeros_like(x_vec)
    # next_x_vec[0] = x_vec[0] + x_vec[1]
    # next_x_vec[1] = x_vec[1] + u_vec[0] - 0.1 * x_vec[1] ** 2

    a_mat = np.array([[1, 0.1],
                     [-1, 2]])
    b_mat = np.array([[0.2, 1],
                     [0.5, 2]])
    next_x_vec = a_mat @ x_vec + b_mat @ u_vec
    return next_x_vec


def cost(x_vec, u_vec):
    """
    cost function
    :param x_vec: 当前状态
    :param u_vec: 控制输入
    :return: cost
    """
    q_mat = np.diag([1, 1])     # State cost matrix
    r_mat = np.diag([1, 1])     # Control cost matrix
    # print('x_vec', x_vec, '\r\n', 'u_vec', u_vec, '\r\n')
    return x_vec.T @ q_mat @ x_vec + u_vec.T @ r_mat @ u_vec


# Prediction horizon

# if __name__ == '__main__':
#     # Create an instance of the controller
#     ex_x_bounds = [(-100, 100), (-100, 100)]  # State bounds
#     ex_u_bounds = [(-10, 10), (-10, 10)]  # Control bounds
#     non_mpc = NonlinearMPCController(dynamics, cost, 8, 2, 2, ex_x_bounds, ex_u_bounds)
#     # Simulate the system
#     x0 = np.array([[2],
#                    [2]])
#     x_history = [x0]
#     u_history = []
#     time_steps = 50
#     time_used = 0
#     for _ in range(time_steps):
#         start_time = time.time()
#         ut = non_mpc.predict_control(x_history[-1])
#         x_next = dynamics(x_history[-1], ut)
#         x_history.append(x_next)
#         u_history.append(ut)
#         time_used += time.time() - start_time
#     print(f"Average time taken per iteration: {time_used / time_steps:.6f}s")
#
#     # Convert history to numpy arrays for plotting
#     x_history = np.array(x_history)
#     u_history = np.array(u_history)
#
#     # Plotting the results
#     import matplotlib.pyplot as plt
#
#     plt.figure(figsize=(12, 5))
#
#     plt.subplot(1, 2, 1)
#     plt.plot(x_history[:, 0], label='State x1')
#     plt.plot(x_history[:, 1], label='State x2')
#     plt.title('State Trajectories')
#     plt.legend()
#     plt.subplot(1, 2, 2)
#     plt.plot(u_history[:, 0], label='Control u1')
#     plt.plot(u_history[:, 1], label='Control u2')
#     plt.title('Control Inputs')
#     plt.legend()
#
#     plt.tight_layout()
#     plt.show()
#
