import sys
sys.path.append("../../utilities/")
from utils_polynomials import extract_data_from_polynomial
from utils import (
    generate_path,
    save_data,
    save_dict,
    np
    )
from pydrake.all import (
    Saturation,
    LogVectorOutput,
    TrajectorySource,
    PiecewisePolynomial
)
from pydrake.systems.primitives import Multiplexer, Demultiplexer
from pydrake.systems.controllers import PidController
def load_pid_controller(plant, context, builder, x0, u0, pid_gains, tau_limit):
    Kp, Ki, Kd= pid_gains
    pid = PidController(Kp, Kd, Ki)
    # Adding the controller to the builder
    regulator = builder.AddSystem(pid)
    regulator.set_name('PID_controller')

    # Connection of plant and controller
    saturation = builder.AddSystem(Saturation(min_value=[-1 * tau_limit], max_value=[tau_limit]))
    saturation.set_name('saturation')
    builder.Connect(regulator.get_output_port_control(), saturation.get_input_port())
    builder.Connect(saturation.get_output_port(), plant.get_actuation_input_port())
    # Spliting the states
    demux_state = builder.AddSystem(Demultiplexer(4 , 1))
    demux_state.set_name('state_splitter')
    builder.Connect(plant.get_state_output_port(), demux_state.get_input_port(0))
    mux_elbow = builder.AddSystem(Multiplexer(2))
    mux_elbow.set_name('elbow_states')
    # extract theta2
    builder.Connect(demux_state.get_output_port(1), mux_elbow.get_input_port(0)) 
    # extract theta2_dot
    builder.Connect(demux_state.get_output_port(3), mux_elbow.get_input_port(1)) 
    # feed [theta2, theta2_dot] to PD    
    builder.Connect(mux_elbow.get_output_port(), regulator.get_input_port_estimated_state())

    
    # Desired state for PID controller
    desired_trajectory = builder.AddSystem(TrajectorySource(x0))
    desired_trajectory.set_name('desired_states')
    
    # extract desired vectors
    demux_desired = builder.AddSystem(Demultiplexer(4 , 1))# spliting the states
    demux_desired.set_name('desired_state_splitter')
    builder.Connect(desired_trajectory.get_output_port(0), demux_desired.get_input_port(0))

    mux_elbow_desired = builder.AddSystem(Multiplexer(2))
    mux_elbow_desired.set_name('mux_elbow_desired')
    builder.Connect(demux_desired.get_output_port(1), mux_elbow_desired.get_input_port(0)) # extract theta2
    builder.Connect(demux_desired.get_output_port(3), mux_elbow_desired.get_input_port(1)) # extract theta2_dot
    builder.Connect(mux_elbow_desired.get_output_port(), regulator.get_input_port_desired_state())# feed [theta2, theta2_dot] to PD    `
    # Save data loggers for sates and input torque
    input_logger = LogVectorOutput(saturation.get_output_port(0), builder)
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    hyper_params_dict = {
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd,
        "Trajectory duration(seconds)":x0.end_time()
    }
    return (
     pid,    
     hyper_params_dict,
     builder,
     state_logger,
     input_logger
    )

def save_trajectory(
    maneuver, 
    x_trajectory, 
    u_trajectory, 
    frequency, 
    hyper_params, 
    controller, 
    meas_state, 
    meas_torque
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
            meas_state.data()[0,:],
            meas_state.data()[1,:],
            meas_state.data()[2,:],
            meas_state.data()[3,:],
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
    trajectory_folder = "data/trajectories/closed_loop_pid"
    up_directory = 4
    file_name = f"{maneuver}.csv"
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
    save_data(data, trajectory_path)
    file_name = f"{maneuver}.txt"
    params_path = generate_path(trajectory_folder, file_name, up_directory)
    save_dict(hyper_params, params_path)
    return data    