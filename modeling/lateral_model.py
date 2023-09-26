import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Car:
    """Define a class for collecting car states of lateral model at each step"""

    def __init__(self, cr:float, cf:float, m: float, v:float, lr:float, lf:float, Iz:float, ts:float) -> None:
        """Initialize the parameters and states of the lateral model of a car"""
        self.v = v
        self.m = m
        self.cr = cr
        self.cf = cf
        self.lr = lr
        self.lf = lf
        self.Iz = Iz
        self.ts = ts

        self.y = 0
        self.beta = 0
        self.psi = 0
        self.psi_dot = 0

    def reset(self) -> None:
        """Reset state variables of the lateral model"""
        self.y = 0
        self.beta = 0
        self.psi = 1
        self.psi_dot = 0

    def step(self, delta:float) -> None:
        """Update the car states in the lateral modeling
        delta: steering input
        """
        self.y += self.y + (self.beta * self.v + self.v * self.psi) * self.ts
        self.beta += self.beta + ((-(self.cr * self.cf) * self.beta /(self.m * self.v)) + ((self.cr * self.lr - self.cf * self.lf) / (self.m * (self.v ** 2)) - 1) * self.psi_dot + (self.cf * delta/ (self.m * self.v))) * self.ts
        self.psi += self.psi + (self.psi_dot) * self.ts
        self.psi_dot += self.psi_dot + (((self.cr * self.lr - self.cf * self.lf) / self.Iz) * self.beta + (-(self.cr * (self.lr ** 2) + self.cf * (self.lf ** 2)) / (self.Iz * self.v)) * self.psi_dot) * self.ts


    def normal_plot(self, x, t) -> None:
        """Plot the final car trajectory"""
        plt.plot(x, t, label='Lateral position in time')
        plt.legend()
        plt.show()
    

    def animated_plot(self, x, t) -> None:
        """Plot the animated car trajectory"""
        frame = 10
        fig, ax = plt.subplots()
        line, = ax.plot(x, t, label='Lateral position in time')

        def animate(i):
            line.set_ydata(t[:i*frame])
            line.set_xdata(x[:i*frame])
            return line,
        anim = animation.FuncAnimation(fig, animate, frames=10000, interval=1)
        ax.legend()
        writergif = animation.PillowWriter(fps=30)
        anim.save(f'./modeling/images/lateral-trajectory.gif', writer=writergif)
        plt.close()


    