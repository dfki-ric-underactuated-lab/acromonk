from cProfile import label
import sys

sys.path.append("../../utilities/")
from utils import (
    drake_visualizer,
    create_acromonk_plant,
    np,
    load_desired_trajectory,
)
from utils_plot import make_results_directory, plot_custom_data_with_dir
sys.path.append("controllers/tvlqr/")
from tvlqr_utils import (
    load_tvlqr_controller,
    TvlqrControllerSystem,
    save_trajectory
)
plant, context, scene_graph, builder = create_acromonk_plant()
maneuver = input(
    "Enter the name of atomic behavior (ZB, ZF, BF, FB): "
).upper()
assert maneuver in (["ZB", "ZF", "BF", "FB"])
x0, u0, _, _ =  load_desired_trajectory(maneuver)
from pydrake.all import (
    Saturation,
    LogVectorOutput
)
# TVLQR hyper parameters
Q = np.diag([10,10,1,1])
R = np.eye(1)*2
Qf = 10 * Q
# Load the TVLQR controller
tvlqr, hyper_params = load_tvlqr_controller(plant, context, x0, u0, Q, R, Qf)
# Connect the diagram for simulation 
tau_limit = 3
saturation = builder.AddSystem(Saturation(min_value=[-1 * tau_limit], max_value=[tau_limit]))
builder.Connect(saturation.get_output_port(0), plant.get_actuation_input_port())
controller = builder.AddSystem(TvlqrControllerSystem(plant, tvlqr))
builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))    
# Save data loggers for sates and input torque
input_logger = LogVectorOutput(saturation.get_output_port(0), builder)
state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
# Visualize the closed-loop control
duration = x0.end_time()
intial_state = x0.value(x0.start_time())
simulator = drake_visualizer(scene_graph, builder,initial_state=intial_state ,duration=x0.end_time())
# Save the simulation results and hyper parameters
input_log = input_logger.FindLog(simulator.get_context())
state_log = state_logger.FindLog(simulator.get_context())
traj_data = save_trajectory(
    maneuver,
    x0,
    u0,
    frequency=1000,
    hyper_params=hyper_params,
    controller=tvlqr,
    meas_state=state_log,
    meas_torque=input_log
)
# Plot the simulation results
directory = make_results_directory(maneuver, 'tvlqr')
plot_custom_data_with_dir(directory, traj_data, show=False)