# Path Tracker

The path tracker generetes two control signals to follow the desired path. One control signal for the seering angle and one for the velocity are genereted. The steering angle is computed with Pure Pursuit and the velocity command is computed with PID controller, see the subsections below. The code for the path tracker can be found in /svea_starter/src/svea_core/src/svea/controllers/pure_pursuit.py. 

## Steering Controller

Pure Pursuit is used to compute the steering angle. It does so, by continuously looking where the car is at its planned trajectory (which consists of a set of target points) and computes a steering angle towards a target point ahead. How far in the trajectory it will look to compute the steering angle, can be tuned by the variable look- ahead distance Lfc and look forward gain k. 

## Velocity Controller

To compute the velocity command u, a PID controller is used:

<img src="https://render.githubusercontent.com/render/math?math= u = K_p*e + K_i*e_sum + K_d*dedt">

where K_p, K_i and K_d are the proportional, integral and derivative gains. The variable e is the error between the target velocity and the actual velocity of the car at time t. The sum e_sum, represents the discrete form of the integral of the error up to time t, which can be represented as a discrete sum of the erros up to time t:

<img src="https://render.githubusercontent.com/render/math?math= \sum_{n=0}^{t} edt">

The variable dedt, represents the derivative of the error in time t, which is calculated with backward differention:

<img src="https://render.githubusercontent.com/render/math?math= dedt = (e_t - e_{t-1})/dt"> 

### Switching Target Velocity

To follow the path more accurately, especially in curves, the controller switches target velocities depending if the car drives on a straight path or a curve. The controller calculates the angle between the car's yaw and the target position two steps ahead. If the angle exceeds a certain threshold, then the system assumess that a curve will appear and switches to a lower target velocity. 

### Saturation

The controller also uses saturation for the control signal u. This is to make sure that the signal does not exceed a certain limit, which can cause too large overshoot of the car's velocity. 
