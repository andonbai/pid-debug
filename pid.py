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
    
    def update(self, feedback_value, dt, sacle):
        error = self.setpoint - feedback_value
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.previous_error) / dt
        output = (self.Kp * error + self.Ki * self.integral + self.Kd * derivative) * sacle
        output = max(min(output, self.integral_limit), -self.integral_limit)
        self.previous_error = error
        return output

# Simulation parameters
dt = 1
sacle = 0.1
time = np.arange(0, 10, dt)
setpoint = 4200  # Desired wheel speed in units per second
feedback = 2100   # Initial wheel speed

# PID parameters
Kp = 10
Ki = 5
Kd = 0
integral_limit = 4000

# Create PID controller
pid = PID(Kp, Ki, Kd, setpoint, integral_limit)

# Data storage for plotting
feedback_list = []
output_list = []

# Simulation loop
for t in time:
    output = pid.update(feedback, dt, sacle)
    feedback += output * dt  # simple system model
    feedback_list.append(feedback)
    output_list.append(output)

# Plot results
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(time, feedback_list, label='Wheel Speed')
plt.plot(time, [setpoint]*len(time), 'r--', label='Setpoint')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Speed (units/s)')

plt.subplot(2, 1, 2)
plt.plot(time, output_list, label='PID Output')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('PID Output')

plt.tight_layout()
plt.show()
