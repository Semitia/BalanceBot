import numpy as np
import matplotlib.pyplot as plt
import do_mpc
import time

# 定义模型
model_type = 'discrete'  # 离散模型
model = do_mpc.model.Model(model_type)

# 定义状态变量
x1 = model.set_variable(var_type='_x', var_name='x1', shape=(1,1))
x2 = model.set_variable(var_type='_x', var_name='x2', shape=(1,1))

# 定义控制输入
u1 = model.set_variable(var_type='_u', var_name='u1', shape=(1,1))

# 定义系统方程
dx1_dt = -x2 + u1
dx2_dt = x1*x2
model.set_rhs('x1', dx1_dt)
model.set_rhs('x2', dx2_dt)

# 完成模型设置
model.setup()

# 创建MPC实例
mpc = do_mpc.controller.MPC(model)
setup_mpc = {
    'n_horizon': 10,
    't_step': 0.1,
    'n_robust': 1,
    'store_full_solution': False,
}
mpc.set_param(**setup_mpc)

# 目标函数
mterm = x1**2 + x2**2  # Terminal cost
lterm = x1**2 + x2**2  # Stage cost
mpc.set_objective(mterm=mterm, lterm=lterm)
mpc.set_rterm(u1=0.01)  # 加权控制输入成本
# 设置IPOPT不打印
mpc.set_param(nlpsol_opts={'ipopt.print_level':0, 'print_time':0})
# 定义控制器的约束
mpc.bounds['lower', '_u', 'u1'] = -5.0
mpc.bounds['upper', '_u', 'u1'] = 5.0

# 完成控制器设置
mpc.setup()

# 创建模拟环境
simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step=0.1)
simulator.setup()

# 初始化状态
x0 = np.array([[0.5], [1.5]])
u0 = np.array([[0.0]])
simulator.x0 = x0
mpc.x0 = x0
print("Initial state x0 before simulation:", simulator.x0, mpc.x0)

# 准备存储数据
x1_traj = [float(x0[0, 0])]
x2_traj = [float(x0[1, 0])]
u_traj = [float(u0[0, 0])]
# 执行模拟和控制
for step in range(20):
    start_time = time.time()  # 循环开始时的时间

    u0 = mpc.make_step(x0)
    x0 = simulator.make_step(u0)
    x1_traj.append(float(x0[0, 0]))  # 从二维数组x0中提取单个元素
    x2_traj.append(float(x0[1, 0]))  # 同上
    print(f"Step {step+1}: x0 = {x0.flatten()}")
    u_traj.append(float(u0[0, 0]))
    print(f"Control input u0 = {u0.flatten()}")

    end_time = time.time()  # 循环结束时的时间
    elapsed_time = end_time - start_time
    print(f"Time elapsed for step {step+1}: {elapsed_time:.6f} seconds")


# 绘制状态变化轨迹 和 控制输入
time_steps = list(range(len(x1_traj)))
# 创建一个图形窗口和三个子图
fig, axs = plt.subplots(3, 1, figsize=(10, 9))  # 3行1列
# 绘制 x1 的轨迹
axs[0].plot(time_steps, x1_traj, label='x1', marker='o')
axs[0].set_title('Trajectory of x1 over Time')
axs[0].set_xlabel('Time Step')
axs[0].set_ylabel('x1')
axs[0].grid(True)
axs[0].legend()
# 绘制 x2 的轨迹
axs[1].plot(time_steps, x2_traj, label='x2', marker='x')
axs[1].set_title('Trajectory of x2 over Time')
axs[1].set_xlabel('Time Step')
axs[1].set_ylabel('x2')
axs[1].grid(True)
axs[1].legend()
# 绘制 u 的轨迹
axs[2].plot(time_steps, u_traj, label='u', marker='^')
axs[2].set_title('Control Input (u) over Time')
axs[2].set_xlabel('Time Step')
axs[2].set_ylabel('u')
axs[2].grid(True)
axs[2].legend()

# 调整子图间的间距
plt.tight_layout()

# 显示图表
plt.show()

# 显示图表
plt.show()

# 绘制 x2 的轨迹
plt.figure(figsize=(10, 3))
plt.plot(time_steps, x2_traj, label='x2', marker='x')
plt.title('Trajectory of x2 over Time')
plt.xlabel('Time Step')
plt.ylabel('x2')
plt.grid(True)
plt.legend()

# 显示图表
plt.show()

# 绘制 u 的轨迹
plt.figure(figsize=(10, 3))
plt.plot(time_steps, u_traj, label='u', marker='^')
plt.title('Control Input (u) over Time')
plt.xlabel('Time Step')
plt.ylabel('u')
plt.grid(True)
plt.legend()

# 显示图表
plt.show()

