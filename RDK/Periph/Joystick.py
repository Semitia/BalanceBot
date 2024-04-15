import pygame


class JoystickCtrl:
    def __init__(self):
        # 初始化pygame和joystick模块
        pygame.init()
        pygame.joystick.init()
        # 检查是否有手柄连接
        self.joystick_count = pygame.joystick.get_count()
        if self.joystick_count == 0:
            raise OSError('No joysticks detected')
        # 初始化第一个手柄
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def read_input(self):
        # 更新事件队列，这一步是必须的
        pygame.event.pump()
        # 读取手柄的轴和按钮状态
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
        return axes, buttons

    def close(self):
        # 在程序结束时关闭手柄
        if self.joystick:
            self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()


if __name__ == '__main__':
    # 示例使用
    controller = JoystickCtrl()
    while True:
        ex_axes, ex_buttons = controller.read_input()
        print(f"Axes: {ex_axes}")
        print(f"Buttons: {ex_buttons}")
        # 这里可以加上一段延时来限制读取的频率，例如 pygame.time.wait(100)

