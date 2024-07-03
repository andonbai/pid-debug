import numpy as np
import matplotlib.pyplot as plt

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, integral_limit):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit
        self.integral = 0
        self.previous_error = 0
    
    def update(self, feedback_value, dt, scale):
        error = self.setpoint - feedback_value
        self.integral += error
        # Limiting the integral term
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.previous_error)
        output = (self.Kp * error + self.Ki * self.integral + self.Kd * derivative) * scale
        # Limiting the output
        output = max(min(output, self.integral_limit), -self.integral_limit)
        self.previous_error = error
        return output

# Simulation parameters
dt = 0.02  # time step in seconds
scale = 0.1  # scale factor for PID output
time_end = 1  # simulation duration in seconds
time = np.arange(0, time_end, dt)
setpoint_rpm = 60  # Desired wheel speed in RPM (for example)

# Physical and system parameters
wheel_radius = 0.0325  # meters
wheel_base = 0.219  # meters
max_linear_acceleration = 3.0  # m/s²
max_angular_acceleration = 2.5  # rad/s²

# Encoder and PWM parameters
encoder_counts_per_revolution = 2256
max_pwm_value = 4500

# Convert setpoint RPM to encoder counts per second
setpoint_counts_per_second = 1.22 * encoder_counts_per_revolution

# PID parameters
Kp = 10
Ki = 6
Kd = 0
integral_limit = max_pwm_value

# Create PID controller
pid = PID(Kp, Ki, Kd, setpoint_counts_per_second, integral_limit)

# Data storage for plotting
feedback_list = []
output_list = []

# Initial feedback in encoder counts per second
feedback_counts_per_second = 0

# Simulation loop
for t in time:
    output_pwm = pid.update(feedback_counts_per_second, dt, scale)
    
    # Limit the output PWM based on physical constraints
    output_pwm = np.clip(output_pwm, -max_pwm_value, max_pwm_value)
    
    # Simulate feedback (simple model: output directly affects speed)
    feedback_counts_per_second += output_pwm
    
    feedback_list.append(feedback_counts_per_second)
    output_list.append(output_pwm)

# Plot results
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(time, feedback_list, label='Wheel Speed (Counts per Second)')
plt.plot(time, [setpoint_counts_per_second]*len(time), 'r--', label='Setpoint')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Speed (Counts per Second)')

plt.subplot(2, 1, 2)
plt.plot(time, output_list, label='PID Output (PWM)')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('PID Output (PWM)')

plt.tight_layout()
plt.show()
