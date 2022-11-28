import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import namedtuple
from utils_rpi import generate_path, os
import sys
sys.path.append('../utilities/')
from piecewise_polynomials import FitPiecewisePolynomial
from datetime import datetime


def prepare_store_data(n):
    (
        shoulder_meas_pos,
        shoulder_meas_vel,
        elbow_meas_pos,
        elbow_meas_vel,
        elbow_meas_tau,
        elbow_cmd_tau,
        elbow_clipped_tau,
        raw_imu_pos,
        raw_imu_vel,
        meas_time,
        _,
        _        
    ) = prepare_empty_arrays(n)
    (
        shoulder_des_pos_store,
        shoulder_des_vel_store,
        elbow_des_pos_store,
        elbow_des_vel_store,
        elbow_des_tau_store,
        k1_store,
        k2_store,
        k3_store,
        k4_store,
        state_machine_flag,
        _,
        _
    ) = prepare_empty_arrays(n)
    Data = namedtuple(
        "Data",
        [
            "meas_time",
            "shoulder_des_pos_store",
            "shoulder_meas_pos",
            "shoulder_des_vel_store",
            "shoulder_meas_vel",
            "elbow_des_pos_store",
            "elbow_meas_pos",
            "elbow_des_vel_store",
            "elbow_meas_vel",
            "elbow_des_tau_store",
            "elbow_meas_tau",
            "elbow_cmd_tau",
            "elbow_clipped_tau",
            "raw_imu_pos",
            "raw_imu_vel",
            "k1_store",
            "k2_store",
            "k3_store",
            "k4_store",
            "state_machine_flag",            
        ],
    )
    data = Data(
        shoulder_meas_pos=shoulder_meas_pos,
        shoulder_meas_vel=shoulder_meas_vel,
        elbow_meas_pos=elbow_meas_pos,
        elbow_meas_vel=elbow_meas_vel,
        elbow_meas_tau=elbow_meas_tau,
        elbow_cmd_tau=elbow_cmd_tau,
        elbow_clipped_tau=elbow_clipped_tau,
        raw_imu_pos=raw_imu_pos,
        raw_imu_vel=raw_imu_vel,
        meas_time=meas_time,
        shoulder_des_pos_store=shoulder_des_pos_store,
        shoulder_des_vel_store=shoulder_des_vel_store,
        elbow_des_pos_store=elbow_des_pos_store,
        elbow_des_vel_store=elbow_des_vel_store,
        elbow_des_tau_store=elbow_des_tau_store,
        k1_store=k1_store,
        k2_store=k2_store,
        k3_store=k3_store,
        k4_store=k4_store,
        state_machine_flag=state_machine_flag,        
    )
    return data


def prepare_des_data(csv_data):
    ## shoulder
    des_time = csv_data["time"].values
    shoulder_des_pos = csv_data["shoulder_pos"].values
    shoulder_des_vel = csv_data["shoulder_vel"].values
    shoulder_des_acc = csv_data["shoulder_acc"].values
    shoulder_des_jerk = csv_data["shoulder_jerk"].values
    shoulder_des_tau = csv_data["shoulder_torque"].values
    ## elbow.values
    elbow_des_pos = csv_data["elbow_pos"].values
    elbow_des_vel = csv_data["elbow_vel"].values
    elbow_des_acc = csv_data["elbow_acc"].values
    elbow_des_jerk = csv_data["elbow_jerk"].values
    elbow_des_tau = csv_data["elbow_torque"].values
    elbow_des_tau_tvlqr = csv_data["elbow_torque_tvlqr"].values
    # K values
    k1 = csv_data["K1"].values
    k2 = csv_data["K2"].values
    k3 = csv_data["K3"].values
    k4 = csv_data["K4"].values
    k0 = csv_data["k0"].values

    # converting the desired trajectories according to the gear ratio
    dt = csv_data["time"][2] - csv_data["time"][1]
    Data = namedtuple(
        "Data",
        [
            "shoulder_des_pos",
            "shoulder_des_vel",
            "shoulder_des_acc",
            "shoulder_des_jerk",
            "shoulder_des_tau",
            "elbow_des_pos",
            "elbow_des_vel",
            "elbow_des_tau",
            "elbow_des_acc",
            "elbow_des_jerk",
            "elbow_des_tau_tvlqr",
            "des_time",
            "k1",
            "k2",
            "k3",
            "k4",
            "k0",
        ],
    )
    data = Data(
        shoulder_des_pos=shoulder_des_pos,
        shoulder_des_vel=shoulder_des_vel,
        shoulder_des_acc=shoulder_des_acc,
        shoulder_des_jerk=shoulder_des_jerk,
        shoulder_des_tau=shoulder_des_tau,
        elbow_des_pos=elbow_des_pos,
        elbow_des_vel=elbow_des_vel,
        elbow_des_acc=elbow_des_acc,
        elbow_des_jerk=elbow_des_jerk,
        elbow_des_tau=elbow_des_tau,
        elbow_des_tau_tvlqr=elbow_des_tau_tvlqr,
        des_time=des_time,
        k1=k1,
        k2=k2,
        k3=k3,
        k4=k4,
        k0=k0,
    )
    return data


def prepare_empty_arrays(n):
    shoulder_meas_pos = np.empty(n)
    shoulder_meas_vel = np.empty(n)
    shoulder_meas_tau = np.empty(n)
    shoulder_cmd_tau = np.empty(n)
    elbow_meas_pos = np.empty(n)
    elbow_meas_vel = np.empty(n)
    elbow_meas_tau = np.empty(n)
    elbow_cmd_tau = np.empty(n)
    elbow_clipped_tau = np.empty(n)
    raw_imu_pos = np.empty(n)
    raw_imu_vel = np.empty(n)
    meas_time = np.empty(n)
    shoulder_meas_pos[:] = np.nan
    shoulder_meas_vel[:] = np.nan
    shoulder_meas_tau[:] = np.nan
    shoulder_cmd_tau[:] = np.nan
    elbow_meas_pos[:] = np.nan
    elbow_meas_vel[:] = np.nan
    elbow_meas_tau[:] = np.nan
    elbow_cmd_tau[:] = np.nan
    elbow_clipped_tau[:] = np.nan
    raw_imu_pos[:] = np.nan
    raw_imu_vel[:] = np.nan
    meas_time[:] = np.nan
    return (
        shoulder_meas_pos,
        shoulder_meas_vel,
        shoulder_meas_tau,
        shoulder_cmd_tau,
        elbow_meas_pos,
        elbow_meas_vel,
        elbow_meas_tau,
        elbow_cmd_tau,
        elbow_clipped_tau,
        raw_imu_pos,
        raw_imu_vel,
        meas_time,
    )


def read_data(folder, file, up_directory):
    path_to_file = generate_path(folder, file, up_directory)
    data = pd.read_csv(path_to_file)
    return data


def save_data(data, path_to_save):
    header = ",".join(data.keys())
    data = np.vstack(data.values()).T
    np.savetxt(path_to_save, data, delimiter=",", header=header, comments="")
    print(f"saved csv to {path_to_save}")


def data_append(
    DATA,
    phase,
    index,
    TIME=np.nan,
    SH_MEAS_POS=np.nan,
    SH_MEAS_VEL=np.nan,
    EL_MEAS_POS=np.nan,
    EL_MEAS_VEL=np.nan,
    EL_CMD_TAU=np.nan,
    EL_MEAS_TAU=np.nan,
    EL_CLIPPED_TAU=np.nan,
    RAW_IMU_POS=np.nan,
    RAW_IMU_VEL=np.nan,
    EL_DES_POS=np.nan,
    EL_DES_VEL=np.nan,
    SH_DES_POS=np.nan,
    SH_DES_VEL=np.nan,
    TVLQR_K1=np.nan,
    TVLQR_K2=np.nan,
    TVLQR_K3=np.nan,
    TVLQR_K4=np.nan,
):
    if phase == "swing":
        DATA.elbow_meas_pos[index] = EL_MEAS_POS
        DATA.elbow_meas_vel[index] = EL_MEAS_VEL
        DATA.elbow_meas_tau[index] = EL_MEAS_TAU
        DATA.elbow_cmd_tau[index] = EL_CMD_TAU
        DATA.elbow_clipped_tau[index] = EL_CLIPPED_TAU
        DATA.shoulder_meas_pos[index] = SH_MEAS_POS
        DATA.shoulder_meas_vel[index] = SH_MEAS_VEL
        DATA.raw_imu_pos[index] = RAW_IMU_POS[0]
        DATA.raw_imu_vel[index] = RAW_IMU_VEL[0]
        # store desired data
        DATA.elbow_des_pos_store[index] = EL_DES_POS
        DATA.elbow_des_vel_store[index] = EL_DES_VEL
        DATA.shoulder_des_pos_store[index] = SH_DES_POS
        DATA.shoulder_des_vel_store[index] = SH_DES_VEL
        DATA.meas_time[index] = TIME
        DATA.k1_store[index] = TVLQR_K1
        DATA.k2_store[index] = TVLQR_K2
        DATA.k3_store[index] = TVLQR_K3
        DATA.k4_store[index] = TVLQR_K4
        DATA.state_machine_flag[index] = 1
    if phase == "recovery":
        DATA.elbow_meas_pos[index] = EL_MEAS_POS
        DATA.elbow_meas_vel[index] = EL_MEAS_VEL
        DATA.elbow_meas_tau[index] = EL_MEAS_TAU
        DATA.elbow_cmd_tau[index] = EL_CMD_TAU
        DATA.elbow_clipped_tau[index] = EL_CLIPPED_TAU
        DATA.shoulder_meas_pos[index] = SH_MEAS_POS
        DATA.shoulder_meas_vel[index] = SH_MEAS_VEL
        DATA.raw_imu_pos[index] = RAW_IMU_POS[0]
        DATA.raw_imu_vel[index] = RAW_IMU_VEL[0]
        # store desired data
        DATA.elbow_des_pos_store[index] = EL_DES_POS
        DATA.elbow_des_vel_store[index] = EL_DES_VEL
        DATA.shoulder_des_pos_store[index] = SH_DES_POS
        DATA.shoulder_des_vel_store[index] = SH_DES_VEL
        DATA.meas_time[index] = TIME
        DATA.k1_store[index] = TVLQR_K1
        DATA.k2_store[index] = TVLQR_K2
        DATA.k3_store[index] = TVLQR_K3
        DATA.k4_store[index] = TVLQR_K4
        DATA.state_machine_flag[index] = 3
    elif phase == "kickback":
        DATA.elbow_meas_pos[index] = EL_MEAS_POS
        DATA.elbow_meas_vel[index] = EL_MEAS_VEL
        DATA.elbow_meas_tau[index] = EL_MEAS_TAU
        DATA.elbow_cmd_tau[index] = EL_CMD_TAU
        DATA.elbow_clipped_tau[index] = EL_CLIPPED_TAU
        DATA.shoulder_meas_pos[index] = SH_MEAS_POS
        DATA.shoulder_meas_vel[index] = SH_MEAS_VEL
        DATA.meas_time[index] = TIME
        DATA.state_machine_flag[index] = 0        
    elif phase == "catch":
        DATA.elbow_meas_pos[index] = EL_MEAS_POS
        DATA.elbow_meas_vel[index] = EL_MEAS_VEL
        DATA.elbow_meas_tau[index] = EL_MEAS_TAU
        DATA.elbow_cmd_tau[index] = EL_CMD_TAU
        DATA.elbow_clipped_tau[index] = EL_CLIPPED_TAU
        DATA.shoulder_meas_pos[index] = SH_MEAS_POS
        DATA.shoulder_meas_vel[index] = SH_MEAS_VEL
        DATA.meas_time[index] = TIME
        DATA.state_machine_flag[index] = 2
    elif phase == "go_to_zero":
        DATA.elbow_meas_pos[index] = EL_MEAS_POS
        DATA.elbow_meas_vel[index] = EL_MEAS_VEL
        DATA.elbow_meas_tau[index] = EL_MEAS_TAU
        DATA.elbow_cmd_tau[index] = EL_CMD_TAU
        DATA.elbow_clipped_tau[index] = EL_CLIPPED_TAU
        DATA.shoulder_meas_pos[index] = SH_MEAS_POS
        DATA.shoulder_meas_vel[index] = SH_MEAS_VEL
        DATA.raw_imu_pos[index] = RAW_IMU_POS[0]
        DATA.raw_imu_vel[index] = RAW_IMU_VEL[0]
        DATA.meas_time[index] = TIME
        DATA.state_machine_flag[index] = 4


def create_nominal_pcws(
    data,
):
    breaks = 25
    poly_degree_states = 2
    sh_des_pos_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.shoulder_des_pos,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    sh_des_vel_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.shoulder_des_vel,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    el_des_pos_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.elbow_des_pos,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    el_des_vel_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.elbow_des_vel,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    el_des_tau_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.elbow_des_tau,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )
    el_tau_tvlqr_pcw = FitPiecewisePolynomial(
        data_x=data.des_time,
        data_y=data.elbow_des_tau_tvlqr,
        num_break=breaks,
        poly_degree=poly_degree_states,
    )

    num_break = 25
    poly_deg = 3

    k0 = FitPiecewisePolynomial(
        data_y=data.k0,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k1 = FitPiecewisePolynomial(
        data_y=data.k1,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k2 = FitPiecewisePolynomial(
        data_y=data.k2,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k3 = FitPiecewisePolynomial(
        data_y=data.k3,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )

    k4 = FitPiecewisePolynomial(
        data_y=data.k4,
        data_x=data.des_time,
        num_break=num_break,
        poly_degree=poly_deg,
    )
    PCSW = namedtuple(
        "PCWS",
        [
            "k0",
            "k1",
            "k2",
            "k3",
            "k4",
            "sh_des_pos_pcw",
            "el_des_pos_pcw",
            "sh_des_vel_pcw",
            "el_des_vel_pcw",
            "el_des_tau_pcw",
            "el_tau_tvlqr_pcw",
        ],
    )
    pcws = PCSW(
        k0=k0,
        k1=k1,
        k2=k2,
        k3=k3,
        k4=k4,
        sh_des_pos_pcw=sh_des_pos_pcw,
        el_des_pos_pcw=el_des_pos_pcw,
        sh_des_vel_pcw=sh_des_vel_pcw,
        el_des_vel_pcw=el_des_vel_pcw,
        el_des_tau_pcw=el_des_tau_pcw,
        el_tau_tvlqr_pcw=el_tau_tvlqr_pcw,
    )

    return pcws


def plot_custom_data_with_dir(directory, data, show=True):
    plot_data(
        x_data=[
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
        ],
        y_data=[
            data["shoulder_meas_pos"],
            data["shoulder_des_pos_store"],
            data["elbow_meas_pos"],
            data["elbow_des_pos_store"],
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
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
            data["meas_time"],
        ],
        y_data=[
            data["shoulder_meas_vel"],
            data["shoulder_des_vel_store"],
            data["elbow_meas_vel"],
            data["elbow_des_vel_store"],
        ],
        labels=["Time (s)", "Velocity (rad/s)"],
        title="Velocity (rad/s) vs Time (s)",
        legends=["sh_meas", "sh_des", "el_meas", "el_des"],
        linestyle=["-", "--", "-", "--"],
        colors=["blue", "cornflowerblue", "red", "indianred"],
        save_name=directory + "/vel.pdf",
    )

    plot_data(
        x_data=[data["meas_time"], data["meas_time"], data["meas_time"]],
        y_data=[
            data["elbow_meas_tau"],
            data["elbow_cmd_tau"],
            data["elbow_clipped_tau"],
        ],
        labels=["Time (s)", "Torque (Nm)"],
        title="Torque (Nm) vs Time (s)",
        legends=["el_meas", "el_cmd", "el_clipped"],
        linestyle=["-", "-", "--"],
        colors=["blue", "red", "aqua"],
        save_name=directory + "/tau.pdf",
    )
    if show:
        plt.show()


def make_results_directory(test):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = f"results/real_system/{test}"
    file_name = date
    directory = generate_path(folder_name, file_name, 3)
    os.makedirs(directory)
    return directory


def plot_custom_data(data, test, show=True):
    directory = make_results_directory(test)
    plot_custom_data_with_dir(directory, data, show)
    return directory


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