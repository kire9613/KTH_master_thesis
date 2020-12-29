# SVEA Team 1

## A short description
This is a working solution for path following and collision detection and avoidance, for use on the SVEA platform.

## Dependencies
Follow the instructions under __Installing the library__ in `README.md` to install missing ROS dependencies and build the package. However, it is also necessary to install python libraries

- `casadi`
- `numpy`
- `matplotlib`
- `scipy`

_Note:_ The SVEA computer is of `aarch64` architecture and `casadi` has to be built from source with `clang` support (for JIT compilation), and optionally also with `ipopt`. Please find instructions on the `casadi` webpage.

## How to run the code
Simulations can be run with
```bash
roslaunch svea_core q1_mpc.launch
```

To run the code on the actual SVEA
```bash
roslaunch svea_core q1_real.launch
```

## MPC Path follower

We use the the ordinary nonlinear bicycle model defined by the ODE

<!--
\large\displaystyle\begin{align*}\dot{x} &= v \cos \psi \\ \dot{y} &= v \sin \psi \\ \dot{\psi} &= \frac{v}{d} \tan \delta \\ \end{align*}
-->
<img src="https://render.githubusercontent.com/render/math?math=%5Clarge%5Cdisplaystyle%0A%5Cbegin%7Balign*%7D%0A%20%20%20%20%5Cdot%7Bx%7D%20%26%3D%20v%20%5Ccos%20%5Cpsi%20%5C%5C%0A%20%20%20%20%5Cdot%7By%7D%20%26%3D%20v%20%5Csin%20%5Cpsi%20%5C%5C%0A%20%20%20%20%5Cdot%7B%5Cpsi%7D%20%26%3D%20%5Cfrac%7Bv%7D%7Bd%7D%20%5Ctan%20%5Cdelta%20%5C%5C%0A%5Cend%7Balign*%7D%0A">

with the variables and parameters defined as seen in the figure below

![Model](docs/.static/model.svg "Model")

The current state of the vehicle, along with references that are dynamically generated at each sampling instance (fig. below), are sent into the MPC which generates a control signal.

![References](docs/.static/references.png "References")
