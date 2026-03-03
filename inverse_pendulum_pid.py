import math
from simple_pid import PID

class InversePendulumPID():
    def __init__(self):
        # internal state variables
        self.theta = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.pid = PID(1.0, 0.1, 0.05, setpoint=0.0)
        self.pid.sample_time = 0.01 #in seconds
        self.length = .7 # in meters

    def control_loop(self, measured_theta: float):
        # Update internal state with measured theta
        self.theta = measured_theta
        
        # Convert theta to an x position
        x_pos = math.sin(self.theta) * self.length
        print(x_pos)
        
        # Get control signal from PID to drive x_pos to 0
        control = self.pid(x_pos)
        
        # Update acceleration and integrate to get velocity
        self.acceleration = control
        self.velocity += self.acceleration * self.pid.sample_time
        
        # Print component info
        print(f"  P (proportional): {self.pid._proportional:.6f}")
        print(f"  I (integral): {self.pid._integral:.6f}")
        print(f"  D (derivative): {self.pid._derivative:.6f}")
        return self.velocity

def reset(self):
    self.velocity = 0
    self.acceleration = 0

def main():
    controller = InversePendulumPID()
    # sample angles in degrees (-90 to 90)
    angles_deg = [-90, -45, 0, 45, 90]
    print("PID controller demo")
    for idx, ang_deg in enumerate(angles_deg, start=1):
        reset(controller)
        ang_rad = math.radians(ang_deg)
        output = controller.control_loop(ang_rad)
        print(f"Step {idx}: theta={ang_deg:.1f}° ({ang_rad:.3f} rad) -> output={output:.3f}")


if __name__ == "__main__":
    main()
        
    