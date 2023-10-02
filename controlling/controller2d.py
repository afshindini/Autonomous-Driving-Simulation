#!/usr/bin/env python3


import cutils
import numpy as np

class Controller2D(object):
    """ 2D Controller Class which implements longitudinal/lateral controllers for tracking reference waypoints in Carla simulator.
        Controller Feedback Variables:
            x               : Current X position (meters)
            y               : Current Y position (meters)
            yaw             : Current yaw pose (radians)
            v               : Current forward speed (meters per second)
            t               : Current time (seconds)
            v_desired       : Current desired speed (meters per second)
                                (Computed as the speed to track at the
                                closest waypoint to the vehicle.)
            waypoints       : Current waypoints to track
                                (Includes speed to track at each x,y
                                location.)
                                Format: [[x0, y0, v0],
                                        [x1, y1, v1],
                                        ...
                                        [xn, yn, vn]]
                                Example:
                                    waypoints[2][1]: 
                                    Returns the 3rd waypoint's y position

                                    waypoints[5]:
                                    Returns [x5, y5, v5] (6th waypoint)
        
        Controller Output Variables:
            throttle_output : Throttle output (0 to 1)
            steer_output    : Steer output (-1.22 rad to 1.22 rad)
            brake_output    : Brake output (0 to 1)
    """

    def __init__(self, waypoints: list) -> None:
        """Initialize state/control variables"""
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi


    def update_values(self, x:float, y:float, yaw:float, speed:float, timestamp:float, frame:int) -> None:
        """Update x/y/yaw/speed/timestamp/frame based on new values"""
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True


    def update_desired_speed(self) -> None:
        """Find the desired speed based on the nearest waypoint to the current location x/y"""
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed


    def update_waypoints(self, new_waypoints:list) -> None:
        """Update waypoints based on new list"""
        self._waypoints = new_waypoints


    def get_commands(self):
        """Return the calculated throttle/steer/brake values"""
        return self._set_throttle, self._set_steer, self._set_brake


    def set_throttle(self, input_throttle: float) -> None:
        """Clamp the throttle command to the valid bounds"""
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle


    def set_steer(self, input_steer_in_rad:float) -> None:
        """Covnert radians to [-1, 1] and clamp the steering command to valid bounds"""
        input_steer = self._conv_rad_to_steer * input_steer_in_rad
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer


    def set_brake(self, input_brake:float) -> None:
        """Clamp the steering command to valid bounds"""
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake


    def update_controls(self) -> None:
        """Update control variables and retrieve simulator feedback"""
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0


        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('e_previous', 0.0)
        self.vars.create_var('e_previous_total', 0.0)


        # Skip the first frame to store previous values properly
        if self._start_control_loop:

            # Longitudinal PID controller
            kP = 1.5
            kI = 0.7
            kD = 0.4

            delta_t = t - self.vars.t_previous  # Calculate time difference for PID controller
            error = v_desired - v   # Calculate velocity error as the input of PID controller
            proportional = kP * error   # Proportional controller
            integral_error = self.vars.e_previous_total + error * delta_t   # Calculate the sum of error based on its previous values
            integral = kI * integral_error  # Integral controller
            derivate = kD * (error - self.vars.e_previous) / delta_t    # Derivative controller
            pid = proportional + integral + derivate    # PID controller output
            if pid >= 0:
                throttle_output = pid
                brake_output = 0
            else:
                throttle_output = 0
                brake_output = -pid

            # Lateral Stanley controller
            # Calculate heading error
            yaw_p = np.arctan2(waypoints[-1][1] - waypoints[0][1], waypoints[-1][0] - waypoints[0][0])
            heading_error = yaw_p - yaw 

            # Calculate crosstrack error
            k_e = 0.3
            crosstrack_error = np.min(np.sum((np.array([x, y]) - np.array(waypoints)[:, :2])**2, axis=1))
            yaw_crosstrack = np.arctan2(y - waypoints[0][1], x - waypoints[0][0])
            if (yaw_p - yaw_crosstrack) > 0:
                crosstrack_error = abs(crosstrack_error)
            else:
                crosstrack_error = - abs(crosstrack_error)
            crosstrack_error = np.arctan(k_e * crosstrack_error / v)

            # Output of Stanely controller
            steer_output = crosstrack_error + heading_error


            # Set control outputs
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t  # Store time to be used in next step
        self.vars.e_previous = error  # Store velocity error to be used in next step
        self.vars.e_previous_total = integral_error  # Store integral velocity error till this timeframe to be used in next step