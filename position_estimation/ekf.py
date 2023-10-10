import pickle

from typing import Any, List
from dataclasses import dataclass, field

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class Utility:
    """Helper functions"""

    def plot(self, x: Any, y:Any, path:str, xlabel:str, ylabel: str, title:str) -> None:
        """Plot and save normal figures"""
        e_fig = plt.figure()
        ax = e_fig.add_subplot()
        ax.plot(x, y)
        ax.set_xlabel(f'{xlabel}')
        ax.set_ylabel(f'{ylabel}')
        ax.set_title(f'{title}')
        plt.savefig(f'{path}')
        plt.show()

    
    def plot_animated(self, x:Any, y:Any, path:str, title:str) -> None:
        """Plot and save animated figures"""
        frame = 10
        fig, ax = plt.subplots()
        line, = ax.plot(x, y, label=f'{title}')

        def animate(i):
            line.set_ydata(y[:i*frame])
            line.set_xdata(x[:i*frame])
            return line,
        anim = animation.FuncAnimation(fig, animate, frames=10000, interval=1)
        ax.legend()
        writergif = animation.PillowWriter(fps=30)
        anim.save(f'{path}', writer=writergif)
        plt.close()



@dataclass
class InitEstimator:
    """Initialize values for the main state estimator"""
    # Data from data file
    data: Any = field(init=False)

    # estimated states and covariances
    x_est: Any = field(init=False)
    P_est: Any = field(init=False)

    # input signal
    v: Any = field(init=False)  # translational velocity input [m/s]
    om: Any = field(init=False) # rotational velocity input [rad/s]

    # bearing and range measurements, LIDAR constants
    b: Any = field(init=False)  # bearing to each landmarks center in the frame attached to the laser [rad]
    r: Any = field(init=False)  # range measurements [m]
    l: Any = field(init=False)  # x,y positions of landmarks [m]
    d: Any = field(init=False)  # distance between robot center and laser rangefinder [m]

    # Variances
    v_var: float = field(init=False, default=0.01)  # translation velocity variance  
    om_var: float = field(init=False, default=0.01)  # rotational velocity variance 
    r_var: float = field(init=False, default=0.01)  # range measurements variance
    b_var: float = field(init=False, default=10)  # bearing measurement variance
    Q_km: Any = field(init=False)   # input noise covariance 
    cov_y: Any = field(init=False)  # measurement noise covariance 

    # First states
    t: float = field(init=False)    # timestamps [s]
    x_init: float = field(init=False)   # initial x position [m]
    y_init: float = field(init=False)   # initial y position [m]
    th_init: float = field(init=False)  # initial theta position [rad]


    def __post_init__(self) -> None:
        """Initialize states"""
        self._read_data()   # read data from file

        self.x_init = self.data['x_init']   # init x
        self.y_init = self.data['y_init']   # init y
        self.th_init = self.data['th_init'] # init theta

        self.b = self.data['b']
        self.d = self.data['d']
        self.l = self.data['l']
        self.r = self.data['r']

        self.v  = self.data['v']
        self.om = self.data['om']

        self.t = self.data['t']

        self._cov_matrix()  # Define the covariance matrices
        self._state_init()  # init estimated states and covariance matrix


    def _read_data(self) -> None:
        """Read data file"""
        with open('./position_estimation/data/data.pickle', 'rb') as f:
            self.data = pickle.load(f)


    def _cov_matrix(self) -> None:
        """Define covariance matrices"""
        self.Q_km = np.diag([self.v_var, self.om_var])  # input noise covariance 
        self.cov_y = np.diag([self.r_var, self.b_var])  # measurement noise covariance 


    def _state_init(self) -> None:
        """Initialize states and covarinaces"""
        self.x_est = np.zeros([len(self.v), 3])
        self.P_est = np.zeros([len(self.v), 3, 3])
        self.x_est[0] = np.array([self.x_init, self.y_init, self.th_init]) # initial estimated state
        self.P_est[0] = np.diag([1, 1, 0.1])    # initial estimated covariance


    def wraptopi(self, x:float) -> None:
        """Wraps angle to (-pi,pi] range"""
        if x > np.pi:
            x = x - (np.floor(x / (2 * np.pi)) + 1) * 2 * np.pi
        elif x < -np.pi:
            x = x + (np.floor(x / (-2 * np.pi)) + 1) * 2 * np.pi
        return x


@dataclass
class PoseEstimator(InitEstimator, Utility):
    """Use Extended Kalman Filter (EKF) to estimate vehicle state/pose"""

    def correction_step(self, lk:Any, rk:Any, bk:Any, P_check:Any, x_check:Any) -> List[Any]:
        """return corrected covariance and predicted states"""
        Hk, Mk, y_out = self.measurement_jacobian(lk, x_check)  # Calculate measurement Jacobian matrices
        Kk = P_check.dot(Hk.T).dot(np.linalg.inv(Hk.dot(P_check).dot(Hk.T) + Mk.dot(self.cov_y).dot(Mk.T)))  # Calculate Kalman gain
        x_check = x_check + Kk.dot(np.vstack([rk, self.wraptopi(bk)]) - y_out)  # Correct predicted states
        x_check[2] = self.wraptopi(x_check[2])  # Wrap angle to [-pi, pi]
        P_check = (np.eye(len(x_check)) - Kk.dot(Hk)).dot(P_check) # Correct covariance 

        return x_check, P_check
    

    def measurement_jacobian(self,lk: Any, x_check: Any) -> List[Any]:
        """Calculate measurement jacobian matrices"""
        xk, yk, thetak = x_check[0], x_check[1], self.wraptopi(x_check[2])
        xl, yl = lk[0], lk[1]
        dx = xl - xk - self.d * np.cos(thetak)
        dy = yl - yk - self.d * np.sin(thetak)
        r = np.sqrt(dx**2 + dy**2)
        phi = np.arctan2(dy, dx) - thetak
        y_out = np.vstack([r, self.wraptopi(phi)])
        Hk = np.zeros((len(lk), len(x_check)))
        Hk[0,0] = -dx/r
        Hk[0,1] = -dy/r
        Hk[0,2] = self.d * (dx * np.sin(thetak) - dy * np.cos(thetak)) / r
        Hk[1,0] = dy / r**2
        Hk[1,1] = -dx / r**2
        Hk[1,2] = -1-self.d * (dy * np.sin(thetak) + dx * np.cos(thetak)) / r**2
        Mk = np.eye(len(lk))

        return Hk, Mk, y_out


    def prediction_step(self) -> None:
        """Implement prediction step or main loop of EKF"""
        # First values are at t=0 based on intialized ones
        P_check = self.P_est[0]
        x_check = self.x_est[0][...,np.newaxis]

        for k in range(1,len(self.t)):
            delta_t = self.t[k] - self.t[k - 1]  # time step (difference between timestamps)
            theta = self.wraptopi(x_check[2])  # Wrap the angle to [-pi, pi]

            F = np.array([[np.cos(theta[0]), 0.0], [np.sin(theta[0]), 0.0], [0.0, 1.0]])
            input = np.array([[self.v[k-1]], [self.om[k-1]]])
            x_check = x_check + F.dot(input) * delta_t    # Update state with odometry readings
            x_check[2] = self.wraptopi(x_check[2])

            F_km = np.zeros([3, 3])
            F_km = np.array([[1.0, 0.0, -np.sin(theta[0]) * delta_t * self.v[k-1]], [0.0, 1.0, np.cos(theta[0]) * delta_t * self.v[k-1]], [0.0, 0.0, 1.0]])   # Motion model jacobian with respect to last state

            L_km = np.zeros([3, 2])
            L_km = np.array([[np.cos(theta[0]) * delta_t, 0.0], [np.sin(theta[0]) * delta_t, 0.0], [0.0, 1.0]])   # Motion model jacobian with respect to noise

            P_check = F_km.dot(P_check.dot(F_km.T)) + L_km.dot(self.Q_km.dot(L_km.T))   # Propagate uncertainty
            
            for i in range(len(self.r[k])):
                x_check, P_check = self.correction_step(self.l[i], self.r[k, i], self.b[k, i], P_check, x_check)   # Update state estimate using available landmark measurements

            self.x_est[k, 0], self.x_est[k, 1], self.x_est[k, 2], self.P_est[k, :, :] = x_check[0], x_check[1], x_check[2], P_check # Set final state predictions for timestep

    
    def plot_results(self) -> None:
        """Plot estimated trajectories"""
        self.plot(self.x_est[:, 0], self.x_est[:, 1], './position_estimation/images/Estimated-YX-trajectory.png', 'x [m]', 'y [m]', 'Estimated Y-X trajectory')
        self.plot(self.t[:], self.x_est[:, 2], './position_estimation/images/Estimated-Theta-trajectory.png', 'Time [s]', 'theta [rad]', 'Estimated Theta trajectory')


    def plot_animated_results(self) -> None:
        """Plot the animated estimated trajectory"""
        self.plot_animated(self.x_est[:, 0], self.x_est[:, 1], './position_estimation/images/Estimated-YX-trajectory.gif', 'Estimated XY Trajectory')
        self.plot_animated(self.t[:], self.x_est[:, 2], './position_estimation/images/Estimated-Theta-trajectory.gif', 'Estimated Theta Trajectory')

        

if __name__ == '__main__':
    estimator = PoseEstimator()
    estimator.prediction_step()
    estimator.plot_results()
    estimator.plot_animated_results()
