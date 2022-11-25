import sys
sys.path.append("direct_collocation/")
from trajopt_utils import (
    create_acromonk_plant,
    trajopt,
    traj_opt_hyper,
    visualize_traj_opt,
)

sys.path.append("../../../utilities/")
from utils import save_trajectory, make_parent_directory
from utils_plot import plot_traj


maneuver = input(
    "Enter the name of atomic behavior (ZB, ZF, BF, FB): "
).upper()
assert maneuver in (["ZB", "ZF", "BF", "FB"])
plant, context, scene_graph = create_acromonk_plant()
dircol_hyper = traj_opt_hyper(maneuver)
(result, dircol, hyper_params) = trajopt(
    plant=plant,
    context=context,
    n=dircol_hyper[0],
    tau_limit=dircol_hyper[1],
    initial_state=dircol_hyper[2],
    theta_limit=dircol_hyper[3],
    speed_limit=dircol_hyper[4],
    ladder_distance=dircol_hyper[5],
    final_state=dircol_hyper[6],
    R=dircol_hyper[7],
    time_panalization=dircol_hyper[8],
    init_guess=dircol_hyper[9],
)
u_trajectory = dircol.ReconstructInputTrajectory(result)
x_trajectory = dircol.ReconstructStateTrajectory(result)
visualize_traj_opt(plant, scene_graph, x_trajectory)
traj_data = save_trajectory(
    maneuver,
    x_trajectory,
    u_trajectory,
    frequency=1000,
    hyper_params=hyper_params,
)
parent_folder = "data/trajectories"
folder_name = f"direct_collocation/{maneuver}"
directory = make_parent_directory(parent_folder, folder_name, up_directory=5)
plot_traj(traj_data, directory, show=True)
