import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation



class Car:
    """Define a class for collecting car states at each step"""

    def __init__(self, ts:float = 0.01, L:float = 2.0, lr:float = 1.2, wmax:float = 1.22) -> None:
        """Initialize the car states with zero values"""
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

        self.L = L
        self.lr = lr
        self.wmax = wmax
        self.ts = ts
    
    def reset(self) -> None:
        """Reset the car states"""
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0


    def step(self, v: float, w: float) -> None:
        """Implementation of vehicle kinematic model with v and omega as inputs"""
        w = min(w, self.wmax)
        
        self.xc = self.xc + v * np.cos(self.theta + self.beta) * self.ts
        self.yc = self.yc + v * np.sin(self.theta + self.beta) * self.ts
        self.theta = self.theta + (v / self.L) * np.cos(self.beta) * np.tan(self.delta) * self.ts
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.L)
        self.delta = self.delta + w * self.ts


class Trajectory(Car):
    """Form the trajectory of a car kinematic model of based on its inputs v and omega"""

    def __init__(self, v: float, t:float, shape:str, ts: float = 0.01, L: float = 2, lr: float = 1.2, wmax: float = 1.22) -> None:
        """initialize the velocity, time, shape of the trajectory"""
        super().__init__(ts, L, lr, wmax)
        self.shape =shape
        self.v = v
        self.t = t
        self.t_data = np.arange(0, self.t, self.ts)
        self.trajectory_check()
    
    
    def trajectory_check(self) -> None:
        """Check if the trajectory shape is defined in the class methods"""
        if self.shape not in ["circle", "square", "spiral", "infinity", "wave"]:
            raise AssertionError(f"Trajectory {self.shape} is not defined. Choose from circle, square. spiral, infinity trajectories.")
        

    def plot_trajectory(self, x, y) -> None:
        """Plot the final car trajectory"""
        plt.axis('equal')
        plt.plot(x, y, label=f'Trajectory of car for {self.shape} route.')
        plt.legend()
        plt.show()
    

    def plot_animated_trajectory(self, x, y) -> None:
        """Plot the animated car trajectory"""
        frame = 10
        fig, ax = plt.subplots()
        ax.axis('equal')
        line, = ax.plot(x, y, label=f'Trajectory of car for {self.shape} route.')

        def animate(i):
            line.set_ydata(y[:i*frame])
            line.set_xdata(x[:i*frame])
            return line,
        anim = animation.FuncAnimation(fig, animate, frames=10000, interval=1)
        ax.legend()
        writergif = animation.PillowWriter(fps=30)
        anim.save(f'./modeling/images/{self.shape}-trajectory.gif', writer=writergif)
        plt.close()




    def circle(self, r:float) -> None:
        """Define a circle trajectory with a specific radius
        r: radius of the circle trajectory
        omega is calulated based on the trajectory in such a way that the car will travel the specified trajectory.
        """
        self.reset()
        x, y = np.zeros_like(self.t_data), np.zeros_like(self.t_data)
        for i in range(self.t_data.shape[0]):
            x[i] = self.xc
            y[i] = self.yc

            if self.delta < np.arctan(self.L/r):
                self.step(self.v, self.wmax)
            else:
                self.step(self.v, 0)

        self.plot_trajectory(x,y)
        self.plot_animated_trajectory(x,y)
              

    def square(self) -> None:
        """Define a square trajectory
        omega is calulated based on the trajectory in such a way that the car will travel the specified trajectory.
        """
        self.reset()
        x, y, w = np.zeros_like(self.t_data), np.zeros_like(self.t_data), np.zeros_like(self.t_data)
        w[670:670+100] = 0.753
        w[670+100:670+100*2] = -0.753
        w[2210:2210+100] = 0.753
        w[2210+100:2210+100*2] = -0.753
        w[3670:3670+100] = 0.753
        w[3670+100:3670+100*2] = -0.753
        w[5220:5220+100] = 0.753
        w[5220+100:5220+100*2] = -0.753

        for i in range(self.t_data.shape[0]):
            x[i] = self.xc
            y[i] = self.yc
            self.step(self.v, w[i])

        self.plot_trajectory(x,y)
        self.plot_animated_trajectory(x,y)
  

    def spiral(self) -> None:
        """Define a spiral trajectory
        omega is calulated based on the trajectory in such a way that the car will travel the specified trajectory.
        """
        self.reset()
        x, y, w = np.zeros_like(self.t_data), np.zeros_like(self.t_data), np.zeros_like(self.t_data)
        w[:] = -1/100
        w[0:100] = 1

        for i in range(self.t_data.shape[0]):
            x[i] = self.xc
            y[i] = self.yc
            self.step(self.v, w[i])

        self.plot_trajectory(x,y)
        self.plot_animated_trajectory(x,y)


    def wave(self) -> None:
        """Define a spiral trajectory
        omega is calulated based on the trajectory in such a way that the car will travel the specified trajectory.
        """
        self.reset()
        x, y, w = np.zeros_like(self.t_data), np.zeros_like(self.t_data), np.zeros_like(self.t_data)
        w[:] = 0
        w[0:100] = 1
        w[100:300] = -1
        w[300:500] = 1
        w[500:5700] = np.tile(w[100:500], 13)
        w[5700:] = -1

        for i in range(self.t_data.shape[0]):
            x[i] = self.xc
            y[i] = self.yc
            self.step(self.v, w[i])

        self.plot_trajectory(x,y)
        self.plot_animated_trajectory(x,y)


    def infinity(self, r:float) -> None:
        """Define a infinity trajectory with radius r
        r: radius of the infinity route
        omega is calulated based on the trajectory in such a way that the car will travel the specified trajectory.
        """
        self.reset()
        x, y = np.zeros_like(self.t_data), np.zeros_like(self.t_data)
        step_change = int(len(self.t_data) / 8)

        for i in range(self.t_data.shape[0]):
            x[i] = self.xc
            y[i] = self.yc
            
            w = self.wmax if (i < step_change or i > step_change * 5) else -self.wmax
            w = 0 if w > 0 and self.delta > 0.99 * np.arctan(self.L/r) else 0 if w < 0 and self.delta < -0.99 * np.arctan(self.L/r) else w
            self.step(self.v, w) 

        self.plot_trajectory(x,y)
        self.plot_animated_trajectory(x,y)


if __name__ == '__main__':
    
    Trajectory(np.pi, 20, "circle").circle(10)   # define a circle trajectory with radius 10, travel time of 20sec and velocity of 3.14m/s.
    Trajectory(4, 60, "square").square()   # define a square trajectory with travel time of 60sec and velocity of 4m/s.
    Trajectory(4, 60, "spiral").spiral()   # define a spiral trajectory with travel time of 60sec and velocity of 4m/s.
    Trajectory(4, 60, "wave").wave()   # define a wave trajectory with travel time of 60sec and velocity of 4m/s.
    Trajectory(np.pi*16/15, 30, "infinity").infinity(8)   # define a infinity trajectory with travel time of 30sec and velocity of 3.349m/s.