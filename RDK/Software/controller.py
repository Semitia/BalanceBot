

class Controller:
    """
    BalanceBot Controller class to handle input from the user.
    Inputs : Cmds from keyboard, joystick, phone, etc.
    Outputs: Unified control signals to the robot
    """
    def __init__(self, device='keyboard'):
        self.device = device

