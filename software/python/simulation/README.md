# Simulation using Mujoco
An installation guide is provided for the simulation of [Reinforcement learning algorithm in mojuco](behavior_generation/reinforcement_learning/README.md).

# Simulation using pydrake
To install pydrake, follow the instructions provided on the official website of [Drake](https://drake.mit.edu/pip.html#stable-releases).

## Trajectory Optimization
To generate atomic behaviors using direct collocation, it is needed to run the python script in the following path:
`software/python/simulation/behavior_generation/trajectory_optimization/simulate_trajopt.py`

by:

```
python simulate_trajopt.py
```

It prompts you the following message to choose from:

`Enter the name of atomic behavior (ZB, ZF, BF, FB, OTHER):`

Typing any of the atomic behaviors, e.g. ZB, will load the default parameters for behavior generation and opens up the simulation with the urdf visualization in a web browser tab. If you want to proceed with arbitrary parameters, type in "OTHER". However, it is necessary to insert the parameters in the script as the following:
```
n               : Number of knot points
tau_limit       : Input torque limit
initial_state   : Initial state: [theta1, theta2, theta1_dot, theta2_dot]
theta_limit     : Position limits for [theta1, theta2]
speed_limit     : Velocity limits for [theta1_dot, theta2_dot]
ladder_distance : Distance between two ladder bars in meter
final_state     : Final state: [theta1, theta2, theta1_dot, theta2_dot]
R               : Input regulization term
W               : Time panalization of the total trajectory
init_guess      : Initial trajectory as initial guess for solver 
```
If a solution exists, then the plots, csv file and hyperparameters will be saved with the same file name that is typed in the `data/trajectories` folder. 

## Trajectory Stabilization

In order to stabilize the obtained trajectory in closed-loop form with TVLQR or PID controller, it is needed to run the python script in the following path:
`software/python/simulation/behavior_control/traj_stabilization.py`

by:

```
python traj_stabilization.py
```
One can provide arbitrary gains for TVLQR and PID controller in the script and analyze the behaviors with different gains. The closed-loop simulation data will be saved in the following folder:
`data/trajectories/closed_loop`

and the plots are saved in the `results/simulation` folder. 

