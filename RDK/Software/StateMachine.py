from transitions import Machine
import time


class StateMachine:
    """
    state machine of wheel-leg balance robot
    """
    states = ['idle', 'chair', 'normal', 'jump', 'emergency']
    transitions = [
        {'trigger': 'to_chair', 'source': 'idle', 'dest': 'chair'},

    ]

    def __init__(self):
        self.machine = Machine(
            model=self,
            states=StateMachine.states,
            transitions=StateMachine.transitions,
            initial='idle'
        )
        self.last_state = 'idle'
        self.trans_ts = time.time()
        self.state = 'idle'         # 虽然machine会帮类注册state示例，但为避免IDE警告，这里再声明一次

    def trans(self, trigger):
        """
        state transition，同时避免IDE对没声明的函数（machine自动注册）的警告
        :param trigger: trigger name
        """
        self.last_state = self.state
        self.trans_ts = time.time()
        self.trigger(trigger)

    # def state_select(self):


if __name__ == '__main__':
    sm = StateMachine()
    print(sm.state)
    sm.trans('to_chair')
    print(sm.state)
