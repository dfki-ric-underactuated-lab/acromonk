# Software Installation
All software has been tested on Ubuntu 20.04 and 22.04. To be able to run all simulation based software, install Drake first into 
a python virtual 
environment, as described on the [Drake website](https://drake.mit.edu/pip.html#stable-releases). After activating the venv, install the remaining 
dependecies via:

    pip install -r requirements.txt

In addition, the training and simulation environment for reinforcement 
learning uses Mujoco 2.1 (see https://github.com/openai/mujoco-py for installation instructions). In order to easily 
find the package, you can add 

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/folder/bin

to your .bashrc. The path is ~/.mujoco/mujoco210/bin by default. 

### Potential Issues

If you face a

    GLEW initialization error: Missing GL version

when trying to start the mujoco simulator, adding 

    export LD_PRELOAD=$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libGLEW.so

to .bashrc can help.


# Trajectory Optimization
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

# Trajectory Stabilization

In order to stabilize the obtained trajectory in closed-loop form with TVLQR or PID controller, it is needed to run the python script in the following path:
`software/python/simulation/behavior_control/traj_stabilization.py`

by:

```
python traj_stabilization.py
```
One can provide arbitrary gains for TVLQR and PID controller in the script and analyze the behaviors with different gains. The closed-loop simulation data will be saved in the following folder:
`data/trajectories/closed_loop`

and the plots are saved in the `results/simulation` folder. 

# Reinforcement Learning

With RL, a closed loop controller can be trained directly in simulation, 
which does not follow a pre-computed trajectory.

To simulate the trained rl controller, move to the behavior_generation/reinforcement_leraning/scripts 
folder and run <code>python replay_rl_model.py</code>. The result should look like this:

<div align="center">
<img width="600" src="../../../hardware/images/bf_rl.gif" />
</div>

More information on training new controllers can be found [here](behavior_generation/reinforcement_learning/README.md).