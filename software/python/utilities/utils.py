import os
import pandas as pd
import numpy as np
from utils_polynomials import (
    fit_polynomial, 
    extract_data_from_polynomial, 
    PiecewisePolynomial, 
    create_gain_arrays
    )

def parent(path):
    return os.path.dirname(path)


def generate_path(path_to_folder, file_name, up_directory_counter):
    cur_dir = os.path.realpath(os.curdir)
    tmp_dir = cur_dir
    i = 0
    while i < up_directory_counter:
        tmp_dir = parent(tmp_dir)
        i += 1
    main_dir = tmp_dir
    return os.path.join(main_dir, path_to_folder, file_name)


def read_data(folder, file, up_directory):
    path_to_file = generate_path(folder, file, up_directory)
    data = pd.read_csv(path_to_file)
    return data


def save_dict(dict, path):
    with open(path, "w") as f:
        for key, value in dict.items():
            f.write(f"{key}={value}\n")
    print(f"Hyperparameters saved in: {path}.")
    with open(path, "r") as f:
        print(f"Hyperparameters:\n{f.read()}")


def save_data(data, path_to_save):
    header = ",".join(data.keys())
    data = np.vstack(list(data.values())).T
    np.savetxt(path_to_save, data, delimiter=",", header=header, comments="")
    print(f"saved trajectory csv to {path_to_save}")


def save_trajectory(
    maneuver, x_trajectory, u_trajectory, frequency, hyper_params, controller_type=None, controller=None, meas_state=None, meas_torque=None
):
    x0_d = x_trajectory.derivative(derivative_order=1)
    x0_dd = x_trajectory.derivative(derivative_order=2)
    # Extract State
    acromonk_state, time_traj = extract_data_from_polynomial(
        x_trajectory, frequency
    )
    # Extract xd_trajectory
    x0_d_vec, _ = extract_data_from_polynomial(x0_d, frequency)
    # Extract xdd_trajectory
    x0_dd_vec, _ = extract_data_from_polynomial(x0_dd, frequency)
    # Extract desired input
    elbow_torque_des, _ = extract_data_from_polynomial(u_trajectory, frequency)
    data = {
        "time": time_traj,
        "shoulder_pos": acromonk_state[0, :],
        "shoulder_vel": acromonk_state[2, :],
        "shoulder_acc": x0_d_vec[2, :],
        "shoulder_jerk": x0_dd_vec[2, :],
        "elbow_pos": acromonk_state[1, :],
        "elbow_vel": acromonk_state[3, :],
        "elbow_acc": x0_d_vec[3, :],
        "elbow_jerk": x0_dd_vec[3, :],
        "elbow_torque": elbow_torque_des[0, :],
    }
    if controller_type==None:
        parent_folder = "data/trajectories"
        folder_name = f"direct_collocation/{maneuver}"
        trajectory_path = make_parent_directory(parent_folder, folder_name, up_directory=5)
        file_name = f"/{maneuver}_traj.csv"
        save_path = trajectory_path + file_name
        save_data(data, save_path)
        params_path = trajectory_path + f"/hyperparameters_{maneuver}.txt"
        save_dict(hyper_params, params_path)
    else:
        # Extract_meas_state
        meas_time = meas_torque.sample_times()
        # des_time = csv_data.time.reshape(csv_data.time.shape[0], -1)
        x_meas_desc = np.vstack(
            (
                meas_state.data()[0, :],
                meas_state.data()[1, :],
                meas_state.data()[2, :],
                meas_state.data()[3, :],
            )
        )
        u_meas_desc = meas_torque.data()
        u_meas_traj = PiecewisePolynomial.FirstOrderHold(meas_time, u_meas_desc)
        x_meas_traj = PiecewisePolynomial.CubicShapePreserving(
            meas_time, x_meas_desc, zero_end_point_derivatives=True
        )
        acromonk_meas_state, time_traj = extract_data_from_polynomial(
            x_meas_traj, frequency
        )
        elbow_torque_meas, _ = extract_data_from_polynomial(u_meas_traj, frequency)
        data["time"] = time_traj
        data["shoulder_pos"] = acromonk_state[0, :]
        data["shoulder_vel"] = acromonk_state[2, :]
        data["shoulder_acc"] = x0_d_vec[2, :]
        data["shoulder_jerk"] = x0_dd_vec[2, :]
        data["elbow_pos"] = acromonk_state[1, :]
        data["elbow_vel"] = acromonk_state[3, :]
        data["elbow_acc"] = x0_d_vec[3, :]
        data["elbow_jerk"] = x0_dd_vec[3, :]
        data["elbow_torque"] = elbow_torque_des[0, :]
        data["shoulder_meas_pos"] = acromonk_meas_state[0, :]
        data["shoulder_meas_vel"] = acromonk_meas_state[2, :]
        data["elbow_meas_pos"] = acromonk_meas_state[1, :]
        data["elbow_meas_vel"] = acromonk_meas_state[3, :]
        data["elbow_meas_torque"] = elbow_torque_meas[0, :]
        if controller_type == "tvlqr":
            K, k0, _ = create_gain_arrays(controller, frequency)
            data["K1"] = (K[..., 0],)
            data["K2"] = (K[..., 1],)
            data["K3"] = (K[..., 2],)
            data["K4"] = (K[..., 3],)
            data["k0"] = (k0,)
        parent_folder = "data/trajectories"
        folder_name = "closed_loop"        
        trajectory_path = make_parent_directory(parent_folder, folder_name, up_directory=4)
        file_name = f"/{maneuver}_{controller_type}.csv"
        save_path = trajectory_path + file_name
        save_data(data, save_path)
        params_path = trajectory_path + f"/hyperparameters_{maneuver}_{controller_type}.txt"
        save_dict(hyper_params, params_path)            
    return data


def forward_kinematics(theta1, theta2, link_length=0.31401):
    """
    Function to compute the forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector position given the joint angles.
    Returns ons the (y,z) coordinates as the AcroMonk can only move in a plane.
    """
    l1 = link_length
    l2 = l1
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta2))
    ee_z = -(l1 * np.cos(theta1) + (l2 * np.cos(theta1 + theta2)))

    return ee_y, ee_z


def forward_diff_kinematics(
    theta1, theta2, theta1_dot, theta2_dot, link_length=0.31401
):
    """
    Function to compute the differential forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector velocity given the join position/velocities
    Returns ons the (y_dot,z_dot) coordinates as the AcroMonk can only move in a plane.
    """
    l1 = link_length
    l2 = l1
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (
        l2 * (theta1_dot + theta2_dot) * np.cos(theta1 + theta2)
    )
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) - (
        l2 * (theta1_dot + theta2_dot) * np.sin(theta1 + theta2)
    )
    return ee_y_dot, ee_z_dot


def create_acromonk_plant():
    from pydrake.all import Parser, DiagramBuilder, AddMultibodyPlantSceneGraph

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    urdf_folder = "data/simulation_models"
    file_name = "acromonk.urdf"
    up_directory = 4
    urdf_path = generate_path(urdf_folder, file_name, up_directory)
    parser.AddModels(urdf_path)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant, context, scene_graph, builder


def drake_visualizer(scene_graph, builder, initial_state, duration, visualize=None):
    from pydrake.all import Simulator, StartMeshcat, MeshcatVisualizer
    print('\n<<<<<Visualization started>>>>>\n')
    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1)
    context_simulator = simulator.get_mutable_context()
    if visualize == 'pid':
        initial_state = np.append(initial_state,[[0]],axis=0)
    if visualize != None:
        context_simulator.SetContinuousState(initial_state)
    visualizer.StartRecording()
    simulator.AdvanceTo(duration)
    visualizer.PublishRecording()
    return simulator


def load_desired_trajectory(maneuver):
    trajectory_folder = f'data/trajectories/direct_collocation/{maneuver}'
    file_name = f'{maneuver}_traj.csv'
    up_directory = 4
    path_to_csv = generate_path(trajectory_folder, file_name, up_directory)
    print(f'[load_desired_trajectory]:loading {path_to_csv}.')
    data_csv = pd.read_csv(path_to_csv)
    x0, u0, x0_d, x0_dd = fit_polynomial(data=data_csv)    
    return x0, u0, x0_d, x0_dd
                     

def make_parent_directory(parent_folder, folder_name, up_directory):
    directory = generate_path(parent_folder, folder_name, up_directory)
    try:
        os.makedirs(directory)
    except BaseException:
        pass
    return directory
