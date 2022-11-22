import os
import pandas as pd
import numpy as np


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
    print(f"Dictonary saved in: {path}.")
    with open(path, "r") as f:
        print(f"Dictonary:\n{f.read()}")


def save_data(data, path_to_save):
    header = ",".join(data.keys())
    data = np.vstack(data.values()).T
    np.savetxt(path_to_save, data, delimiter=",", header=header, comments="")
    print(f"saved csv to {path_to_save}")


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
    urdf_folder = "data/urdf-files/urdf"
    file_name = "acromonk.urdf"
    up_directory = 3
    urdf_path = generate_path(urdf_folder, file_name, up_directory)
    parser.AddModelFromFile(urdf_path)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant, context, scene_graph, builder


def drake_visualizer(plant, scene_graph, builder, duration):
    from pydrake.all import Simulator, ConnectMeshcatVisualizer
    from meshcat.servers.zmqserver import start_zmq_server_as_subprocess

    proc, zmq_url, web_url = start_zmq_server_as_subprocess(server_args=[])
    meshcat = ConnectMeshcatVisualizer(
        builder,
        scene_graph,
        zmq_url=zmq_url,
        delete_prefix_on_load=True,
        open_browser=True,
    )
    meshcat.load()
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1)
    context = simulator.get_mutable_context()
    simulator.Initialize()
    meshcat.start_recording()
    simulator.AdvanceTo(duration)
    meshcat.stop_recording()
    meshcat.publish_recording()
