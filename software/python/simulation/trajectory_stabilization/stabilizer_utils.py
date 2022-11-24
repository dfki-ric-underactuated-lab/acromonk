import sys

sys.path.append("../../utilities/")
from utils_polynomials import extract_data_from_polynomial, create_gain_arrays
from utils import generate_path, save_data, save_dict, np
from pydrake.all import PiecewisePolynomial

sys.path.append("controllers/tvlqr/")
from tvlqr_utils import load_tvlqr_controller

sys.path.append("controllers/pid/")
from pid_utils import load_pid_controller


def load_controller(
    plant,
    context,
    builder,
    des_trajectories,
    controller_type,
    controller_gains,
    tau_limit=3,
):
    pid_gains, tvlqr_gains = controller_gains
    x0, u0 = des_trajectories
    if controller_type == "tvlqr":
        Q, R, Qf = tvlqr_gains
        (
            controller,
            hyper_params_dict,
            builder,
            state_logger,
            input_logger,
        ) = load_tvlqr_controller(
            plant, context, builder, x0, u0, Q, R, Qf, tau_limit
        )
    elif controller_type == "pid":
        (
            controller,
            hyper_params_dict,
            builder,
            state_logger,
            input_logger,
        ) = load_pid_controller(
            plant, context, builder, x0, u0, pid_gains, tau_limit
        )
    return (controller, hyper_params_dict, builder, state_logger, input_logger)


def save_trajectory(
    maneuver,
    controller_type,
    x_trajectory,
    u_trajectory,
    frequency,
    hyper_params,
    controller,
    meas_state,
    meas_torque,
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
    trajectory_folder = "data/trajectories/closed_loop"
    up_directory = 4
    file_name = f"{maneuver}-{controller_type}.csv"
    trajectory_path = generate_path(trajectory_folder, file_name, up_directory)
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
        "shoulder_meas_pos": acromonk_meas_state[0, :],
        "shoulder_meas_vel": acromonk_meas_state[2, :],
        "elbow_meas_pos": acromonk_meas_state[1, :],
        "elbow_meas_vel": acromonk_meas_state[3, :],
        "elbow_meas_torque": elbow_torque_meas[0, :],
    }
    if controller_type == "tvlqr":
        K, k0, _ = create_gain_arrays(controller, frequency)
        data["K1"] = (K[..., 0],)
        data["K2"] = (K[..., 1],)
        data["K3"] = (K[..., 2],)
        data["K4"] = (K[..., 3],)
        data["k0"] = (k0,)
    save_data(data, trajectory_path)
    file_name = f"{maneuver}-{controller_type}.txt"
    params_path = generate_path(trajectory_folder, file_name, up_directory)
    save_dict(hyper_params, params_path)
    return data
