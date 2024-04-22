import pygame
import time


class KeyboardCtrl:
    def __init__(self):
        pygame.init()
        self.key_states = {}
        pygame.display.set_mode((800, 600))  # 设置窗口尺寸

    def read_input(self):
        pygame.event.pump()
        self.key_states = pygame.key.get_pressed()
        return self.key_states

    def close(self):
        pygame.quit()


# 示例用法
if __name__ == '__main__':
    controller = KeyboardCtrl()
    while True:
        keys = controller.read_input()
        print(f"Keys: {keys}")
        if keys[pygame.K_q]:
            break
        time.sleep(1)
    controller.close()
