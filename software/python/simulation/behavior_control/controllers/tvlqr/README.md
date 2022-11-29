#  Time-varying Linear Quadrativ Regulator (TVLQR)

## Theory

This controller is designed to follow a precomputed trajectory. Particulary, the controller can process trajectories that have been found with help of the [behavior generation methods](../../../behavior_generation/README.md). 

The Time-varying Linear Quadratic Regulator (TVLQR) is an extension to the regular [LQR controller](https://underactuated.csail.mit.edu/lqr.html).
The TVLQR controller tries to stabilize the system along a nominal trajectory. For this, at every timestep the system dynamics are linearized around the state of the nominal trajectory $\mathbf{x}_0(t), \mathbf{u}_0(t)$ at the given timestep $t$ as:

```math
\begin{equation*}
\dot{\mathbf{x}} =  \mathbf{A}(t)\mathbf{x} + \mathbf{B}(t)\mathbf{u}
\end{equation*}
```

The LQR formalism then can be used to derive the optimal controller at timestep $t$:
```math
\begin{equation*}
u(\mathbf{x}) = \mathbf{u}_0(t) - \mathbf{K}(t) \left( \mathbf{x} - \mathbf{x}_0(t)\right)
\end{equation*}
```


For further reading, we recommend chapter 8 of this [Underactuated Robotics [1]](http://underactuated.mit.edu/) lecture.

## Dependencies

The trajectory optimization using direct collocation and TVLQR gains are obtained by taking advantage of [Drake toolbox [3]](https://drake.mit.edu/).


## References

[[1] Russ Tedrake. Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (Course Notes for MIT 6.832).](http://underactuated.mit.edu/)

[[2] Model-Based Design and Verification for Robotics](https://drake.mit.edu/).

