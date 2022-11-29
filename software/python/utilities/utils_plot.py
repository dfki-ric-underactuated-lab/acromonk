import matplotlib.pyplot as plt
import os
from datetime import datetime
from utils import forward_kinematics, generate_path, forward_kinematics


def plot_data(
    x_data,
    y_data,
    labels,
    title,
    legends=None,
    save_name=None,
    linestyle=None,
    linewidth=None,
    colors=None,
):
    plt.figure(figsize=(15, 10))
    if linestyle is None:
        linestyle = [None] * len(x_data)
    if colors is None:
        colors = [None] * len(x_data)
    for i in range(len(x_data)):
        plt.plot(
            x_data[i],
            y_data[i],
            linestyle=linestyle[i],
            linewidth=linewidth,
            c=colors[i],
        )

    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.title(title)
    if legends is None:
        pass
    else:
        plt.legend(legends)
    plt.draw()
    if save_name is None:
        pass
    else:
        print(f"Making {save_name} plot.")
        plt.savefig(save_name)
    return plt


def plot_closed_loop_control_data(directory, data, show=True):
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["shoulder_meas_pos"],
            data["shoulder_pos"],
            data["elbow_meas_pos"],
            data["elbow_pos"],
        ],
        labels=["Time (s)", "Position (rad)"],
        title="Position (rad) vs Time (s)",
        legends=["sh_meas", "sh_des", "el_meas", "el_des"],
        linestyle=["-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred"],
        save_name=directory + "/pos.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
            data["time"],
            data["time"],
        ],
        y_data=[
            data["shoulder_meas_vel"],
            data["shoulder_vel"],
            data["elbow_meas_vel"],
            data["elbow_vel"],
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["sh_meas", "sh_des", "el_meas", "el_des"],
        linestyle=["-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred"],
        save_name=directory + "/vel.pdf",
    )

    plot_data(
        x_data=[
            data["time"], 
            data["time"]
            ],
        y_data=[
            data["elbow_meas_torque"],
            data["elbow_torque"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["torque_meas","torque_des"],
        linestyle=["-", "--"],
        colors=["blue", "red"],
        save_name=directory + "/tau.pdf",
    )
    ee_y, ee_z = forward_kinematics(
        data["shoulder_pos"], data["elbow_pos"]
    )
    ee_y_meas, ee_z_meas = forward_kinematics(
        data["shoulder_meas_pos"], data["elbow_meas_pos"]
    )    
    plot_data(
        x_data=[
            ee_y_meas,
            ee_y
            ],
        y_data=[
            ee_z_meas,
            ee_z
        ],
        labels=["Y-component(m)", "Z-component(m)"],
        title="End-Effector Trajectory",
        legends=["measured", "desired"],
        linestyle=["-", "--"],
        colors=["blue", "red"],
        save_name=directory + "/ee_traj.pdf",
    )    
    if show:
        plt.show()


def plot_traj(data, directory, show=True):
    plot_data(
    x_data=[
        data["time"],
        data["time"],
    ],
    y_data=[
        data["shoulder_pos"],
        data["elbow_pos"],
    ],
    labels=["Time (s)", "Position (rad)"],
    title="Position (rad) vs Time (s)",
    legends=["sh_des", "el_des"],
    linestyle=["-", "-"],
    colors=["blue", "red"],
    save_name=directory + "/pos.pdf",
    )
    plot_data(
        x_data=[
            data["time"],
            data["time"],
        ],
        y_data=[
            data["shoulder_vel"],
            data["elbow_vel"],
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["sh_des", "el_des"],
        linestyle=["-", "-"],
        colors=["blue", "red"],
        save_name=directory + "/vel.pdf",
    )

    plot_data(
        x_data=[
            data["time"]
            ],
        y_data=[
            data["elbow_torque"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["torque_des"],
        linestyle=["-"],
        colors=["blue"],
        save_name=directory + "/tau.pdf",
    )
    ee_y, ee_z = forward_kinematics(
        data["shoulder_pos"], data["elbow_pos"]
    )
    plot_data(
        x_data=[
            ee_y
            ],
        y_data=[
            ee_z
        ],
        labels=["Y-component(m)", "Z-component(m)"],
        title="End-Effector Trajectory",
        legends=["ee trajectory"],
        linestyle=["-"],
        colors=["blue"],
        save_name=directory + "/ee_traj.pdf",
    )
    if show:
        plt.show()