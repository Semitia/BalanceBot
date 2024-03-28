import do_mpc


# 状态变量字典
X = {
    's': 0,                 # 位置
    '_s': 1,                # 速度,也即 dot s
    'yaw': 2,               # 偏航角
    '_yaw': 3,              # 偏航角速度
    'th_leg_l': 4,          # 左腿角度
    '_th_leg_l': 5,         # 左腿角速度
    'th_leg_r': 6,          # 右腿角度
    '_th_leg_r': 7,         # 右腿角速度
    'th_body': 8,           # 机体角度
    '_th_body': 9           # 机体角速度
}

# 控制输入字典
U = {
    'u': 0
}


def template_model():
    """
    定义系统的模型
    :return: 返回系统的模型
    """
    # 获取do-mpc的模型类
    model_type = 'discrete'  # 离散模型
    model = do_mpc.model.Model(model_type)
    # 定义状态变量
    # x1 = model.set_variable(var_type='_x', var_name='x1', shape=(1,1))
    # x2 = model.set_variable(var_type='_x', var_name='x2', shape=(1,1))
    states = []
    state_names = ['x1', 'x2']
    for name in state_names:
        states.append(model.set_variable(var_type='_x', var_name=name, shape=(1, 1)))
    # 定义控制输入
    u = []
    ctrl_names = ['u']
    for name in ctrl_names:
        u.append(model.set_variable(var_type='_u', var_name=name, shape=(1, 1)))

    # 状态方程
    # dx1 = x2
    # dx2 = -x1 + u

    # # 设置状态方程
    # model.set_rhs('x1', dx1)
    # model.set_rhs('x2', dx2)

    # 完成模型设置
    model.setup()

    return model


class BotMPC:
    """
    基于do_mpc开发的平衡车MPC控制器
    """
