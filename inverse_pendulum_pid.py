import math
from simple_pid import PID

class InversePendulumPID():
    def __init__(self):
        # internal state variables
        self.measured_ang_acceleration = 0.0
        self.predicted_ang_acceleration = 0.0
        self.measured_angle = 0.0
        self.predicted_angle = 0.0
        self.measured_ang_velocity = 0.0
        self.predicted_ang_velocity = 0.0
        
        self.measured_linear_acceleration = 0.0
        self.corrected_linear_acceleration = 0.0
        # PID controller to normalize acceleration to 0
        self.pid = PID(1.0, 0.1, 0.05, setpoint=0.0)
        self.pid.sample_time = 0.01 #in seconds

    def control_loop(self, measured_linear_acceleration: float):
        # Take in measured acceleration
        self.measured_linear_acceleration = measured_linear_acceleration
        
        # Get corrective control signal from PID to drive acceleration to 0
        corrective_signal = self.pid(measured_linear_acceleration)
        
        # Output the counteracting force in opposite direction
        self.corrected_linear_acceleration = -corrective_signal
        
        # Print component info
        print(f"Measured acceleration: {measured_linear_acceleration:.6f}")
        print(f"  P (proportional): {self.pid._proportional:.6f}")
        print(f"  I (integral): {self.pid._integral:.6f}")
        print(f"  D (derivative): {self.pid._derivative:.6f}")
        print(f"Corrected acceleration: {self.corrected_linear_acceleration:.6f}")
        return self.corrected_linear_acceleration

def reset(self):
    self.measured_acceleration = 0.0
    self.corrected_acceleration = 0.0
    self.pid.reset()

def main():
    controller = InversePendulumPID()
    
    
    #control loop goes here
    #read angle, ang_vel, and ang_acc
    
    #calculate linear_acceleration
    
    #predict linear_acceleration
    
    #use pid to decide the move to correct linear_acceleration
    
    #publish move to robot
    
    accelerations = [-2.5, 0.0, 2.5]
    print("PID acceleration normalization demo\n")
    for idx, accel in enumerate(accelerations, start=1):
        reset(controller)
        output = controller.control_loop(accel)
        print(f"Step {idx}: input_accel={accel:.1f} m/s² -> output_accel={output:.3f} m/s²\n")


if __name__ == "__main__":
    main()
        
    