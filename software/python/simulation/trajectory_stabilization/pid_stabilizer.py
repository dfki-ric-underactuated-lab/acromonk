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
sys.path.append("controllers/pid/")
from pid_utils import (
    load_pid_controller,
    save_trajectory
)
plant, context, scene_graph, builder = create_acromonk_plant()
maneuver = input(
    "Enter the name of atomic behavior (ZB, ZF, BF, FB): "
).upper()
assert maneuver in (["ZB", "ZF", "BF", "FB"])
x0, u0, _, _ =  load_desired_trajectory(maneuver)
# PID gains
Kp = 100 * np.ones(1)
Ki = np.ones(1)
Kd = 2 * np.ones(1)
pid_gains = (Kp, Ki, Kd)
# Load the PID controller
(
 pid, 
 hyper_params, 
 builder, 
 state_logger, 
 input_logger
) = load_pid_controller(plant, context, builder, x0, u0, pid_gains, tau_limit=3)
# Visualize the closed-loop control
duration = x0.end_time()
intial_state = x0.value(x0.start_time())
simulator = drake_visualizer(scene_graph, builder, initial_state=intial_state , duration=x0.end_time(), controller='pid')
# Save the simulation results and hyper parameters
input_log = input_logger.FindLog(simulator.get_context())
state_log = state_logger.FindLog(simulator.get_context())
traj_data = save_trajectory(
    maneuver,
    x0,
    u0,
    frequency=1000,
    hyper_params=hyper_params,
    controller=pid,
    meas_state=state_log,
    meas_torque=input_log
)
# Plot the simulation results
directory = make_results_directory(maneuver, 'pid')
plot_custom_data_with_dir(directory, traj_data, show=True)