import numpy as np
from scipy.linalg import solve_continuous_are
import time

class LQRController:
    def __init__(self, A, B, Q, R):
        """
        初始化LQR控制器

        参数:
        A: 系统动态矩阵
        B: 控制输入矩阵
        Q: 状态权重矩阵
        R: 控制输入权重矩阵
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.K = self.compute_gain()

    def compute_gain(self):
        """
        计算最优状态反馈增益矩阵K
        """
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def control(self, x):
        """
        根据当前状态x计算控制输入

        参数:
        x: 当前系统状态

        返回:
        u: 控制输入
        """
        u = -self.K @ x
        return u


if __name__ == '__main__':
    # 系统参数
    A = np.array([[0, 1], [-1, -1]])
    B = np.array([[0], [1]])
    Q = np.eye(2)  # 状态权重矩阵
    R = np.array([[1]])  # 控制权重矩阵

    # 创建LQR控制器实例
    lqr_controller = LQRController(A, B, Q, R)

    # 假设的当前状态
    x = np.array([1, 1])

    # 计算控制输入
    start_time = time.time()
    u = lqr_controller.control(x)
    print(f"Time taken: {time.time() - start_time:.6f}s")
    print("控制输入u:", u)
