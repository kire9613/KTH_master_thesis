# Controller
The system uses a separate controller for controlling velocity, and a separate controller for controlling steering angle to follow the path.

## Velocity Control
![Velocity controller block diagram](figures/controller_diagram.svg "Velocity controller")

The velocity controller is a PI controller with anti windup shown in the block diagram above. The anti windup was added to handle that the ECS on the SVEA saturates the control signal, and that the control can be overridden with the remote. Using the time shift operator ![equation](https://latex.codecogs.com/svg.latex?%5Cinline%20qx_t%3Dx_%7Bt+1%7D), the controller can be written

![equation](https://latex.codecogs.com/svg.latex?u_t%20%3D%20K_pe_t%20+%20%5Cfrac%7BK_i%7D%7Bq-1%7De_t%20+%20%5Cfrac%7BT%7D%7Bq-1%7D%28u%5Es_t%20-%20u_t%29)

or by expanding the operator as

![equation](https://latex.codecogs.com/svg.svg?u_%7Bt+1%7D%20%3D%20K_pe_%7Bt+1%7D%20+%20%28K_i-K_p%29e_t%20+%20%281-T%29u_t%20+%20Tu%5Es_t)

The benefit of using this form of anti-windup instead of simply disabling the integral part once it has passed a threshold is that the controller determines by itself when anti windup has to be enabled. If the threshold is applied manually there might be feasible references that the system can not track since the threshold has been put too tight.

## Steering Control
The path the car has to follow consists of an ordered list of points. The steering controller looks a distance ahead on the path and steers towards that point. This is the same controller that was provided by the course as an example.
