import sys
import numpy as np
import time
import threading
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


class Oscilloscope:
    def __init__(self, title='实时波形监视'):
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.win.setWindowTitle(title)
        self.plot_num = 0
        self.plots = []          # 画布列表
        self.curve_num = []      # 画布对应的曲线数量
        self.curves = [[], []]   # 画布对应的曲线列表
        # self.timer = QtCore.QTimer()
        # self.timer.timeout.connect(self.update_timer)

    def add_plot(self, title='数据曲线'):
        plot = self.win.addPlot(title=title)
        self.plot_num += 1
        self.curve_num.append(0)
        self.plots.append(plot)

    def add_curve(self, plot_index=0, pen='r'):
        if plot_index >= self.plot_num:
            print('Error: plot_index out of range!')
            return
        curve = self.plots[plot_index].plot(pen=pen)
        self.curve_num[plot_index] += 1
        self.curves[plot_index].append(curve)

    # def update_timer(self):
    #     global example_data
    #     for i in range(3):
    #         example_data[i][:-1] = example_data[i][1:]
    #         example_data[i][-1] = np.random.rand()
    #     self.update_curve(0, 0, example_data[0])
    #     self.update_curve(1, 0, example_data[1])
    #     self.update_curve(1, 1, example_data[2])

    def update_curve(self, plot_index=0, curve_index=0, data=None):
        if plot_index >= self.plot_num or curve_index >= self.curve_num[plot_index]:
            print('Error: plot_index or curve_index out of range!')
            return
        self.curves[plot_index][curve_index].setData(data)

    # def start_timer(self, interval=50):
    #     self.timer.start(interval)

    def run(self):
        if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
            self.app.exec()


def data_update(osc):
    example_data = [np.random.rand(100) for _ in range(3)]
    ang = 0
    while True:
        ang += 0.1
        for i in range(3):
            example_data[i][:-1] = example_data[i][1:]
            example_data[i][-1] = np.sin(ang + i * np.pi / 3)
        if ang > 2 * np.pi:
            ang = 0
        osc.update_curve(0, 0, example_data[0])
        osc.update_curve(1, 0, example_data[1])
        osc.update_curve(1, 1, example_data[2])
        time.sleep(0.05)


# 使用示例
if __name__ == '__main__':
    ex_osc = Oscilloscope()
    ex_osc.add_plot(title='画布1')
    ex_osc.add_curve(plot_index=0, pen='r')
    ex_osc.add_plot(title='画布2')
    ex_osc.add_curve(plot_index=1, pen='g')
    ex_osc.add_curve(plot_index=1, pen='b')
    # osc.start_timer(interval=50)

    data_thread = threading.Thread(target=data_update, args=(ex_osc,))
    data_thread.start()
    ex_osc.run()
    # osc.display_loop()

    # example_data = [np.random.rand(100) for _ in range(3)]
    # while True:
    #     for i in range(3):
    #         example_data[i][:-1] = example_data[i][1:]
    #         example_data[i][-1] = np.random.rand()
    #     osc.update_curve(0, 0, example_data[0])
    #     osc.update_curve(1, 0, example_data[1])
    #     osc.update_curve(1, 1, example_data[2])
    #     time.sleep(0.1)

