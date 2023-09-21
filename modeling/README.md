# Vehicle Kinematic/Dynamic Modeling

The first step in the field of autonomous driving is understanding of kinematic and dynamic modelings of cars.

Kinematic modeling is the process of modeling of a car considering geometric constraints that defines its motions. In the kinemtaic modeling the effects of forces would be neglected. It is good to mention that kinematic modeling has good accuracy in low speeds. 

Dynamic modeling on the other hand, considers the effects of forces and moments on the vehicle and estimates the vehicle motion thoroughout the vehicles operating range based on the newton third rule.

## Kinematic Model of a Vehicle

If you consider a car as a a simple two-wheeled vehicle like a bicycle, then the kinematic model of the car can be described with following equations:

<p>
  <img alt="img-name" src="./images/kinematic_model.png" align="right">
</p>

$$\begin{align*}
\dot{x}_c &= v \cos{(\theta + \beta)} \\
\dot{y}_c &= v \sin{(\theta + \beta)} \\
\dot{\theta} &= \frac{v \cos{\beta} \tan{\delta}}{L} \\
\dot{\delta} &= \omega \\
\beta &= \tan^{-1}(\frac{l_r \tan{\delta}}{L})
\end{align*}$$

where the inputs are the steering angle rate $\omega$ and the vehicle velocity $v$. State variables are the $\theta$ as the heading angle, $\delta$ is steering angle, and $x_c$ and $y_c$ are the movements of the gravity center of the car in the related $XY$ directions.

You can find more information regarding how these equations are derived from this [link](https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357) or this [video](https://www.youtube.com/watch?v=D4AgX1zjx54).


### Kinematic Model Simulation

The car kinematic model is defined in `kinematic_model.py` and in the class `Car`. The `Trajectory` class contains methods for traveing in different types of trajectories such as circle, spiral, wave, square and infinity routes. By running the related code, you would understand how kinematic model of a car works in different situations. The results are shown as following:

<p align="center">
    <img src="./images/circle-trajectory.gif" title="circle" width="250" height="250"/>
    <img src="./images/square-trajectory.gif" title="square" width="250" height="250"/> 
    <img src="./images/spiral-trajectory.gif" title="spiral" width="250" height="250"/> 
</p>

<p align="center">
    <img src="./images/wave-trajectory.gif" title="wave" width="250" height="250"/>
    <img src="./images/infinity-trajectory.gif" title="infinity" width="250" height="250"/> 
</p>


## Dynamic Longitudinal Vehicle Model

The dynamic modeling analyzes the effects of forces on the vehicle. Longitudinal modeling gets the throttle inputs from the vehicle as the throttle percentage $x_{\theta}\in[0,1]$ which provides torque to the engine and subsequently accelerates the vehicle in the longitudinal direction. It is good to mention that the throttle inputs transfer energy from the engine to the torque converter, then to transmission equipment, and finally to the wheels. By using the third rule of newton, these stages can be bundled together in a single inertia term in the following equations:

$$\begin{align}
    J_e \dot{\omega}_e &= T_e - (GR)(r_{eff} F_{load}) \\ m\ddot{x} &= F_x - F_{load}
\end{align}$$

where $J_e$ is the bundled inertia, $T_e$ is the engine torque, $GR$ is the gear ratio, 

