# Reinforcement Learning
## Installation
This installation instruction assumes Ubuntu 20.04. 
We recommend using a dedicated python virtual environment, e.g. using 

    python3 -m venv path/to/your/new/venv

The training and simulation environment uses Mujoco 2.1, which has to be installed first. In order to easily 
find the package, you can add 

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/folder/bin

to your .bashrc. The path is ~/.mujoco/mujoco210/bin by default (see https://github.com/openai/mujoco-py 
for more instructions). 

The requirements can be installed from the file located in 
reinforcement_learning/requirements.txt (e.g. 
<code>pip install -r requirements.txt</code>)


Hint: if you face an 

    GLEW intialization error: Missing GL version

adding 

    export LD_PRELOAD=$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libGLEW.so

to .bashrc can help.

To simulate the trained rl controller, move to the /scripts 
folder and run <code>python replay_rl_model.py</code>. The result should be something like this:

<div align="center">
<img width="600" src="../images/bf_rl.gif" />
</div>

## Training
Training a new controller is done via

    python scripts/train_rl_model.py

The parameters for training and simulation will be loaded from 
the parameters.json file in /training. The reward function 
is modified by changing the weights in parameters.json under the 
"reward_setup" keyword. Rewards with a weight of 0 do not 
take effect. The reward "reward_name" expects a method 
"_reward_reward_name" of evironment/acromonk, so if you want to 
define a new reward, you have to set the weight in the parameters
file and implement the respective method in the acromonk 
environment. 

The current (dense) reward setup looks like this:

<div align="center">
<img width="600" src="../images/reward_visualization_modified_smaller_YZ.png" />
</div>

The trained controller will be saved in 
../data/trained_controllers under a folder with 
the current time, along with a copy of the used 
parameters for reproducibility. You can visualize your trained 
controller by changing the path in scripts/replay_rl_model.py in line 15. 
A specific checkpoint can be chosen by the evaluation 
steps, e.g. 
<code>rl_checkpoint = '10000'</code>. 
(default 'best' uses the best performing agent). 

---
**Challenges:**
- Can you train a controller that achieves BF brachiation in one swing?
- Can you train a BF controller only using enegy based reward?
- Can you train a BF controller only using sparse rewards?
- Can you train controllers for the other atomic behaviors FB/ZB/ZF ?
---