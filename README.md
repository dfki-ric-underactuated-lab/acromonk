# AcroMonk: A Minimalist Underactuated Brachiating Robot
<div align="center">
<img width="500" src="hardware/poster_new.png" />
</div>



<div align="center">
<img width="225" src="images/zb.gif" >
<img width="225" src="images/zf.gif" >
<img width="225" src="images/fb.gif" >
<img width="225" src="images/bf.gif" >
</div>

## Description
This project offers an open-source and low-cost kit to test control algorithms for underactuated robots. 
it implements a **brachiation robot** platform called **AcroMonk** which can brachiate robustly over a horizontally laid ladder bar. AcroMonk is the simplest possible underactuated brachiator built using one quasi-direct drive actuator (QDD) and passive grippers. This is the first brachiator with unactuated grippers that can perform more than two brachiation maneuvers. This project offers different control methods for trajectory stabilization which can be studied using the kit. Additionally, it provides a list of components, discusses best practices for implementation, and presents results from experiments with the simulator and the real system. This repository describes the hardware (CAD, Bill Of Materials (BOM) etc.) required to build the physical system and provides the software (URDF models, simulation and controllers) to control it.

## Reinforcement Learning
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

To simulate the trained rl controller, move to the reinforcement_learning/scripts 
folder and run replay_rl_model.py. The result should be something like this:

<div align="center">
<img width="600" src="images/bf_rl.gif" />
</div>
