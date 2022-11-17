# AcroMonk: A Minimalist Underactuated Brachiating Robot

<embed src="images/2d-arm-acrm-coordinate_v2.pdf" type="application/pdf">

<object data="images/2d-arm-acrm-coordinate_v2.pdf" type="application/pdf" width="100%"> </object>

## Overview
bla

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

    