import sys
sys.path.append("../../utilities/")
from utils import (
    drake_visualizer,
    create_acromonk_plant,
    np,
    load_desired_trajectory,
    save_trajectory,
    make_parent_directory
)
from utils_plot import plot_closed_loop_control_data
from stabilizer_utils import load_controller
from datetime import datetime


plant, context, scene_graph, builder = create_acromonk_plant()
maneuver = input(
    "Enter the name of atomic behavior (ZB, ZF, BF, FB, OTHER): "
).upper()
assert maneuver in (["ZB", "ZF", "BF", "FB", "OTHER"])
controller_type = input("Enter controller type (pid, tvlqr): ").lower()
assert controller_type in (["tvlqr", "pid"])
x0, u0, _, _ = load_desired_trajectory(maneuver)
# Desired trajectories
des_trajectories = (x0, u0)
# PID gains
Kp = 100 * np.ones(1)
Ki = np.ones(1)
Kd = 2 * np.ones(1)
pid_gains = (Kp, Ki, Kd)
# TVLQR hyper parameters
Q = np.diag([10, 10, 1, 1])
R = np.eye(1) * 2
Qf = 10 * Q
tvlqr_gains = (Q, R, Qf)
# Controller gains
controller_gains = (pid_gains, tvlqr_gains)
# Load the controller
(
    controller,
    hyper_params,
    builder,
    state_logger,
    input_logger,
) = load_controller(
    plant=plant,
    context=context,
    builder=builder,
    des_trajectories=des_trajectories,
    controller_type=controller_type,
    controller_gains=controller_gains,
    tau_limit=3,
)
# Visualize the closed-loop control
duration = x0.end_time()
intial_state = x0.value(x0.start_time())
simulator = drake_visualizer(
    scene_graph,
    builder,
    initial_state=intial_state,
    duration=x0.end_time(),
    visualize=controller_type,
)
# Save the simulation results and hyper parameters
input_log = input_logger.FindLog(simulator.get_context())
state_log = state_logger.FindLog(simulator.get_context())
traj_data = save_trajectory(
    maneuver=maneuver,
    x_trajectory=x0,
    u_trajectory=u0,
    frequency=1000,
    hyper_params=hyper_params,
    controller_type=controller_type,    
    controller=controller,
    meas_state=state_log,
    meas_torque=input_log,
)
# Plot the simulation results
parent_folder = f"results/simulation/{maneuver}_{controller_type}"
folder_name = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
directory = make_parent_directory(parent_folder, folder_name, up_directory=4)
plot_closed_loop_control_data(directory, traj_data, show=True)
