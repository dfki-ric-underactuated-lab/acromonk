from mujoco_py import load_model_from_path, MjSim
from utils.paths_handling import project_root_dir
import os

def make_mujoco_simulator(sim_parameters):
    # make the mujoco simulator
    model = load_model_from_path(os.path.join(project_root_dir, 'data', 'robot_model', '20220510_ShwingBotV1.xml'))
    model.opt.timestep = sim_parameters['dt']
    sim = MjSim(model)

    return sim

