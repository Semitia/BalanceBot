import pygame


class GameControl:
    def __init__(self):
        pygame.init()
        self.running = True
        self.key_action_map = {
            pygame.K_w: lambda: print("W键被按下"),
            pygame.K_a: lambda: print("A键被按下"),
            pygame.K_s: lambda: print("S键被按下"),
            pygame.K_d: lambda: print("D键被按下"),
            pygame.K_UP: lambda: print("向上键被按下"),
            pygame.K_DOWN: lambda: print("向下键被按下"),
            pygame.K_LEFT: lambda: print("向左键被按下"),
            pygame.K_RIGHT: lambda: print("向右键被按下"),
            pygame.K_1: lambda: print("1键被按下"),
            pygame.K_2: lambda: print("2键被按下"),
            pygame.K_3: lambda: print("3键被按下"),
            pygame.K_4: lambda: print("4键被按下"),
            pygame.K_LCTRL: lambda: print("Ctrl键被按下"),
            pygame.K_RCTRL: lambda: print("Ctrl键被按下"),
            pygame.K_LSHIFT: lambda: print("Shift键被按下"),
            pygame.K_RSHIFT: lambda: print("Shift键被按下")
        }

    def process_events(self):
        """处理所有的按键和其他事件, 没有按键时，几乎不会占用CPU资源"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                action = self.key_action_map.get(event.key)
                if action:
                    action()

    def run(self):
        """主循环，控制游戏的执行"""
        while self.running:
            self.process_events()

        pygame.quit()


# 使用示例
if __name__ == "__main__":
    game = GameControl()
    game.run()
