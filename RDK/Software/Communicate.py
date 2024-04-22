from Periph.USB2CAN import USB2CAN


# dict : msg id -> device name
msg2name = {
    0x101: "LEFT_FRONT_KNEE",
    0x102: "LEFT_BACK_KNEE",
    0x103: "RIGHT_FRONT_KNEE",
    0x104: "RIGHT_BACK_KNEE",
    0x105: "LEFT_WHEEL",
    0x106: "RIGHT_WHEEL",
    0x301: "ARM_SEND1",
    0x302: "ARM_SEND2",
    0x310: "TOF",
    0x320: "IMU"
}


class CommDevice:
    """
    statu of communication device
    """
    def __init__(self, name, comm_type):
        self.name = name
        self.comm_type = comm_type
        self.online = False
        self.last_recv_ts = 0
        self.recv_num = 0
        self.last_send_ts = 0
        self.send_num = 0


class Communicate:
    """
    communication monitor of a robot,
    """
    def __init__(self):
        self.devices = []

    def add_device(self, name, comm_type):
        """
        add a device to the communication monitor
        :param name: device name
        :param comm_type: communication type
        :return:
        """
        self.devices.append(CommDevice(name, comm_type))

    def recv_msg_update(self, msg_id):
        """
        update the communication status of device corresponding to the received message
        :return:
        """
        pass
