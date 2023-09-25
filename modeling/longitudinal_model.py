import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation



class Car:
    """Define a class for collecting car states of longitudinal model at each step"""

    def __init__(self) -> None:
        """Initialize the parameters and states of the longitudinal model of a car"""
        # Set parameters for conversion of throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002

        # Set gear ratio, effective radius, mass, and inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81

        # Set aerodynamic and friction coefficient
        self.c_a = 1.36
        self.c_r1 = 0.01

        # Set tire force
        self.c = 10000
        self.F_max = 10000

        # Set state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0

        self.ts = 0.01


    def reset(self) -> None:
        """Reset state variables of the longitudinal model"""
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0


    def step(self, throttle, alpha) -> None:
        """Update the car states in the longitudinal modeling
        throttle: x_theta as the throttle percentage
        alpha: as the the incline angle of the road
        """
        # Calculate required parameters to be used in the longitudinal equations
        w_w = self.GR * self.w_e
        s = (w_w * self.r_e - self.v) / self.v
        F_x = self.c * s if abs(s) < 1 else self.F_max
        F_g = self.m * self.g * np.sin(alpha)
        R_x = self.c_r1 * self.v
        F_aero = self.c_a * (self.v ** 2)
        F_load = F_aero + R_x + F_g
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * (self.w_e ** 2))

        # Implement the longitudinal equations
        self.a = (F_x - F_load) / self.m
        self.w_e_dot = (T_e - self.GR * self.r_e * F_load) / self.J_e

        # Update x, v and w_e states based on the sampling time
        self.x += self.v * self.ts - 0.5 * self.a * (self.ts ** 2)
        self.v += self.a * self.ts
        self.w_e += self.w_e_dot * self.ts


    def normal_plot(self, x, t) -> None:
        """Plot the final car trajectory"""
        plt.plot(x, t, label='Longitudinal position in time')
        plt.legend()
        plt.show()
    

    def animated_plot(self, x, t) -> None:
        """Plot the animated car trajectory"""
        frame = 10
        fig, ax = plt.subplots()
        line, = ax.plot(x, t, label='Longitudinal position in time')

        def animate(i):
            line.set_ydata(t[:i*frame])
            line.set_xdata(x[:i*frame])
            return line,
        anim = animation.FuncAnimation(fig, animate, frames=10000, interval=1)
        ax.legend()
        writergif = animation.PillowWriter(fps=30)
        anim.save(f'./modeling/images/longitudinal-trajectory.gif', writer=writergif)
        plt.close()
    

    def sample_trajectory(self) -> None:
        """Longitudinal model simulation for the sample trajectory"""
        travel_time = 20
        t = np.arange(0,travel_time,self.ts)
        x = np.zeros_like(t)
        self.reset()

        for i in range(t.shape[0]):
            throttle = 0.0006*i+0.2 if i <= 500 else 0.5 if i <= 1500 else -0.001*i+2
            alpha = np.arctan(0.05) if self.x <= 60 else np.arctan(0.1) if self.x <= 150 else 0  
            x[i] = self.x
            self.step(throttle, alpha)

        self.normal_plot(t, x)
        self.animated_plot(t, x)

if __name__ == '__main__':

    Car().sample_trajectory()