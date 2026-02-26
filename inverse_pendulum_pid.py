import math
from simple_pid import PID

class InversePendulumPID():
    def __init__(self):
        theta = 0
        velocity = 0
        acceleration = 0
        pid = PID(1,0.1, 0.05, setpoint=1)

    def control_loop(self):
        