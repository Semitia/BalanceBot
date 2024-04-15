"""
五连杆机构VMC解算库
参考链接 https://zhuanlan.zhihu.com/p/613007726
"""
import numpy as np
import matplotlib.pyplot as plt


class VMC:
    def __init__(self, length):
        self.len = length    # 五杆长度,0:虚拟杆
        self.phi = [0] * 6   # 0:虚拟杆
        self.plot = False    # 是否绘图

    def sensor_update(self, phi1, phi4):
        """
        更新传感器数据
        """
        self.phi[1] = phi1
        self.phi[4] = phi4

    def plot_switch(self, on_off):
        """
        绘图开关
        """
        self.plot = on_off

    def fore_kine(self):
        """
        前向运动学解算,B点为坐标系原点
        """
        xb = self.len[1]*np.cos(self.phi[1])
        xd = self.len[4]*np.cos(self.phi[4]) + self.len[5]
        yb = self.len[1]*np.sin(self.phi[1])
        yd = self.len[4]*np.sin(self.phi[4])
        a0 = 2*self.len[2]*(xd - xb)
        b0 = 2*self.len[2]*(yd - yb)
        l_bd = np.sqrt((xd - xb)**2 + (yd - yb)**2)
        c0 = self.len[2]**2 - self.len[3]**2 + l_bd**2
        self.phi[2] = 2 * np.arctan2((b0+np.sqrt(a0**2 + b0**2 - c0**2)), (a0+c0))
        xc = self.len[1]*np.cos(self.phi[1]) + self.len[2]*np.cos(self.phi[2])
        yc = self.len[1]*np.sin(self.phi[1]) + self.len[2]*np.sin(self.phi[2])
        self.len[0] = np.sqrt(((xc - self.len[5]/2)**2)+(yc**2))
        self.phi[0] = np.arctan2(yc, xc - self.len[5]/2)
        self.phi[3] = np.arctan2((yc - yd), (xc - xd))
        # 绘图
        if self.plot:
            xa = 0
            ya = 0
            xe = self.len[5]
            ye = 0
            plt.figure(figsize=(8, 6))
            plt.plot([xa, xb], [ya, yb], 'bo-')  # 杆1
            plt.plot([xb, xc], [yb, yc], 'ro-')  # 杆2
            plt.plot([xc, xd], [yc, yd], 'go-')  # 杆3
            plt.plot([xd, xe], [yd, ye], 'co-')  # 杆4
            plt.plot([xa, xe], [ya, ye], 'mo-')  # 杆5
            plt.plot([self.len[5]/2, xc], [0, yc], 'k--') # 虚拟杆

            # 设置图表属性
            plt.xlim(-0.5, max(self.len) + 0.5)
            plt.ylim(-0.5, max(self.len) + 0.5)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Five-bar Mechanism')
            plt.grid(True)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.show()

    def vmc_solve(self, f, tp):
        """
        由期望虚拟杆输出解算两关节电机扭矩
        :param f:  期望虚拟杆支持力
        :param tp: 期望虚拟杆扭矩
        :return:t  两关节电机扭矩
        """
        fc = -tp/self.len[0]    # 虚拟杆末端切向力
        ft = f - fc             # 虚拟杆末端法向力
        # 旋转矩阵
        r = np.array([[np.cos(self.phi[0]-np.pi/2), -np.sin(self.phi[0]-np.pi/2)],
                      [np.sin(self.phi[0]-np.pi/2),  np.cos(self.phi[0]-np.pi/2)]])
        f_ct = np.array([[fc], [ft]])
        f_xy = np.dot(r, f_ct)
        sigma12 = np.sin(self.phi[1]-self.phi[2])
        sigma23 = np.sin(self.phi[2]-self.phi[3])
        sigma34 = np.sin(self.phi[3]-self.phi[4])
        # 雅克比矩阵
        j = np.array([[ self.len[1]*np.sin(self.phi[3])*sigma12/sigma23,  self.len[4]*np.sin(self.phi[2])*sigma34/sigma23],
                      [-self.len[1]*np.cos(self.phi[3])*sigma12/sigma23, -self.len[4]*np.cos(self.phi[2])*sigma34/sigma23]])
        t = np.dot(j.T, f_xy)    # 关节扭矩
        return t

    def fore_kine_test(self, _phi4, len0):
        """
        测试计算phi0
        :param _phi4:
        :param len0:
        :return:
        """
        xd = self.len[5]/2 + self.len[4]*np.cos(_phi4)
        yd = self.len[4]*np.sin(_phi4)
        dis_d = np.sqrt(xd**2 + yd**2)
        phi0_0 = np.arcsin(np.sin(np.pi-self.phi[4])/dis_d*self.len[4])
        phi0_1 = np.arccos((self.len[3]**2 - dis_d**2 - len0**2)/(2*dis_d*len0))
        return phi0_0+phi0_1


if __name__ == '__main__':
    # 测试
    vmc = VMC([0, 0.13, 0.24, 0.24, 0.13, 0.15])
    vmc.plot_switch(True)
    vmc.sensor_update(3.57,  np.pi-3.57)
    vmc.fore_kine()
    print("len:", vmc.len[0], "phi:", vmc.phi[0]/np.pi*180)
    print(vmc.vmc_solve(100, 0))

    # 画出关节角度与虚拟杆支持力的关系
    # vmc.plot_switch(False)
    # out_list = []
    # for ang in np.linspace(0.5*np.pi, 1.5*np.pi, 100):
    #
    #     vmc.sensor_update(ang, np.pi-ang)
    #     vmc.fore_kine()
    #     out = vmc.vmc_solve(100, 0)
    #     out_list.append(out[0])
    #
    # plt.plot(np.linspace(0.5*np.pi, 1.5 * np.pi, 100), out_list)
    # plt.xlabel('Angle (rad)')
    # plt.ylabel('Output')
    # plt.title('Output vs Angle')
    # plt.show()

    # # 画出len0不变时sin_phi0和phi4的关系
    # sin_phi0_list = []
    # phi4_list = []
    # for phi4 in np.linspace(-np.pi/8, -np.pi/3, 100):
    #     phi0 = vmc.fore_kine_test(phi4, 0.12)
    #     sin_phi0_list.append(np.sin(phi0))
    #     phi4_list.append(phi4/np.pi*180)
    # plt.figure(figsize=(12, 6))
    # plt.plot(phi4_list, sin_phi0_list, 'x-', color='red', label='phi0')
    # plt.xlabel('phi4 (degrees)')
    # plt.ylabel('sin_phi0')
    # plt.title('The relationship between gravity moment and limit angle when controlling leg length')
    # plt.tight_layout()
    # plt.show()

