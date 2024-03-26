## CAN负载率计算

考虑各类消息的发送端

|     单元      | FOC控制板 | TOF  | ICM  | RDK-FOC | ARM  |
| :-----------: | :-------: | :--: | :--: | :-----: | :--: |
| 数据长度/byte |     8     |  2   | 6x2  |   8+4   | 8x2  |
| 数据频率/kHz  |     1     | 0.5  |  1   |    1    | 0.05 |

***说明：**其中TOF、ICM、机械臂均通过另加一个STM32F1来控制，STM32也挂载在CAN总线上*

一帧can消息的bit数为 $ 44 + 8*len_{data}$ ，由此可以考虑如下分配方案

### CAN1

- **FOC控制板x4：**4个关节电机——0x101~0x104
- **RDK-FOC-1：** 只控制四个电机，需8字节数据——0x100

**负载率**
$$
\kappa = \frac{108 \times 5 \times 10^3}{10^6} \times 100\% = 54\%
$$

### CAN2

- **FOC控制板x2：** 2个轮毂电机——0x105~0x106
- **RDK-FOC-1：** 只控制两个电机，需4字节数据——0x200
- **ARM：** 上位机给出的只是目标姿态，进一步由stm32控制，无需较高频率——0x301~0x302

- **TOF：** 0x310
- **ICM：** 0x320

**负载率**
$$
\kappa = \frac{(108 \times 2 + 76 + 206 \times 0.05 + 60 \times0.5 + 92\times2)\times 10^3}{10^6} \times 100\% = 51.63\%
$$
