import matplotlib.pyplot as plt
import os
from datetime import datetime
from utils import generate_path


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
    if show:
        plt.show()


def make_results_directory(maneuver,test):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = f"results/simulation/{maneuver}-{test}"
    file_name = date
    directory = generate_path(folder_name, file_name, 4)
    os.makedirs(directory)
    return directory