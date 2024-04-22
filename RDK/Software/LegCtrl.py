from Algorithm import VMC


class LegCtrl:
    """
    五连杆机构腿部运动学与控制类。
    在建模中将腿部简化，通过LegCtrl进一步控制
    """
    def __init__(self, length):
        self.vmc = VMC(length)

