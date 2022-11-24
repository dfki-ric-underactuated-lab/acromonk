import sys
sys.path.append("../../utilities/")
from utils_polynomials import extract_data_from_polynomial, create_gain_arrays
from utils import (
    generate_path,
    save_data,
    save_dict,
    np
    )
from pydrake.all import (
    Saturation,
    LogVectorOutput
)
from pydrake.all import VectorSystem, PiecewisePolynomial
class TvlqrControllerSystem(VectorSystem):
    def __init__(self, plant, tvlqr):
        VectorSystem.__init__(self, 4, 1)
        self.tvlqr_obj = tvlqr
    def DoCalcVectorOutput(self, context_simulation, acromonk_state, unused, output):
        trajTime = context_simulation.get_time()
        xbar = acromonk_state - (self.tvlqr_obj.x0.value(trajTime)).reshape(4,)
        output[:] = (self.tvlqr_obj.u0.value(trajTime) - 
                    (self.tvlqr_obj.K.value(trajTime).dot(xbar)) -
                     self.tvlqr_obj.k0.value(trajTime))[0][0]    


def load_tvlqr_controller(plant, context, builder, x0, u0, Q, R, Qf, tau_limit):
    from pydrake.all import (
        FiniteHorizonLinearQuadraticRegulatorOptions, 
        FiniteHorizonLinearQuadraticRegulator, 
        )
    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.x0 = x0
    options.u0 = u0
    Q = Q
    R = R
    options.Qf = Qf
    options.input_port_index = plant.get_actuation_input_port().get_index()
    tvlqr = FiniteHorizonLinearQuadraticRegulator(
        plant,
        context,
        t0=x0.start_time(),
        tf=x0.end_time(),
        Q=Q,
        R=R,
        options=options
        )
    hyper_params_dict = {
        "Q": Q,
        "Qf": Qf,
        "R": R,
        "Trajectory duration(seconds)":x0.end_time()
    }
    # Connect the diagram for simulation 
    saturation = builder.AddSystem(Saturation(min_value=[-1 * tau_limit], max_value=[tau_limit]))
    builder.Connect(saturation.get_output_port(0), plant.get_actuation_input_port())
    controller = builder.AddSystem(TvlqrControllerSystem(plant, tvlqr))
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))
    # Save data loggers for sates and input torque
    input_logger = LogVectorOutput(saturation.get_output_port(0), builder)
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    return (
     tvlqr, 
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
    K, k0, _ = create_gain_arrays(controller, frequency)
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
    trajectory_folder = "data/trajectories/closed_loop_tvlqr"
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
        "K1":K[...,0],
        "K2":K[...,1],
        "K3":K[...,2],
        "K4":K[...,3],
        "k0":k0,
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