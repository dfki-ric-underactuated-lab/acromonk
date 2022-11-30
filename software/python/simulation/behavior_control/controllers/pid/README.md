#  Proportional–Integral–Derivative (PID) Control #

## Theory #

This controller is designed to follow a precomputed trajectory. Particularly, the controller can process trajectories that have been found with help of the [behavior generation methods](../../../behavior_generation/README.md). The torque processed by the PID control terms via:
```math
u(t) = K_p e(t) + K_i \int_0^t e(t') \text{d}t' + K_d \frac{\text{d}e(t)}{\text{d}t}
```
where $e(t)$ is the position error at timestep t. The values are assumed to be in SI units, i.e. time in s, position in rad, velocity in rad/s, and torque in Nm.

