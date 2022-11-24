import sys

sys.path.append("../../utilities/")
from utils import (
    drake_visualizer,
    generate_path,
    forward_kinematics,
    forward_diff_kinematics,
    np,
    save_data,
    save_dict,
)
from utils_polynomials import extract_data_from_polynomial
from pydrake.all import DirectCollocation, PiecewisePolynomial, Solve


def trajopt(
    plant,
    context,
    n,
    tau_limit,
    initial_state,
    theta_limit,
    speed_limit,
    ladder_distance,
    final_state,
    R,
    time_panalization,
    init_guess,
):
    min_timestep = 0.1
    max_timestep = 0.8
    dircol = DirectCollocation(
        plant,
        context,
        num_time_samples=n,
        minimum_timestep=min_timestep,
        maximum_timestep=max_timestep,
        input_port_index=plant.get_actuation_input_port().get_index(),
    )
    dircol.AddEqualTimeIntervalsConstraints()

    ## Constraints

    # Initial Torque
    torque_limit = tau_limit  # N*m.
    u_init = dircol.input(0)
    dircol.AddConstraintToAllKnotPoints(u_init[0] == 0)
    # Torque limit
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)
    initial_state = initial_state

    # Speed limit
    prog = dircol.prog()
    prog.AddBoundingBoxConstraint(
        initial_state, initial_state, dircol.initial_state()
    )
    state = dircol.state()
    speed_limit = speed_limit
    dircol.AddConstraintToAllKnotPoints(state[2] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[2])
    dircol.AddConstraintToAllKnotPoints(state[3] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[3])

    # Collision avoidance for front bar
    ee_y, ee_z = forward_kinematics(state[0], state[1])
    ee_y_dot, ee_z_dot = forward_diff_kinematics(
        state[0], state[1], state[2], state[3]
    )
    front_bar_coordinate = [ladder_distance, 0.0]  # [y, z]
    R_bar_radius = 0.01  # 0.001   #assumed as the radius of the ladder branch
    distanceR = np.sqrt(
        (front_bar_coordinate[0] - ee_y) ** 2
        + (front_bar_coordinate[1] - ee_z) ** 2
    )
    dircol.AddConstraintToAllKnotPoints(R_bar_radius <= distanceR)

    # Collision avoidance for back bar
    ee_y, ee_z = forward_kinematics(state[0], state[1])
    ee_y_dot, ee_z_dot = forward_diff_kinematics(
        state[0], state[1], state[2], state[3]
    )
    back_bar_coordinate = [-1 * ladder_distance, 0.0]  # [y, z]
    distanceL = np.sqrt(
        (back_bar_coordinate[0] - ee_y) ** 2
        + (back_bar_coordinate[1] - ee_z) ** 2
    )
    dircol.AddConstraintToAllKnotPoints(R_bar_radius <= distanceL)

    # Elbow angle limits for collision avoidance to attatched bar
    theta_limit = theta_limit
    # Shoulder bounds
    dircol.AddConstraintToAllKnotPoints(state[0] <= theta_limit[0])
    dircol.AddConstraintToAllKnotPoints(-theta_limit[0] <= state[0])
    # Elbow bounds
    dircol.AddConstraintToAllKnotPoints(state[1] <= theta_limit[1])
    dircol.AddConstraintToAllKnotPoints(-theta_limit[1] <= state[1])
    final_state = final_state
    prog.AddBoundingBoxConstraint(
        final_state, final_state, dircol.final_state()
    )

    ## Costs

    # Input
    dircol.AddRunningCost(R * u[0] ** 2)

    # Velocities
    dircol.AddRunningCost(state[2] ** 2)
    dircol.AddRunningCost(state[3] ** 2)

    # Total duration
    dircol.AddFinalCost(dircol.time() * time_panalization)
    # dircol.AddFinalCost(np.sum(dircol.time()) * time_panalization)

    # Initial guess
    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
        init_guess, np.column_stack((initial_state, final_state))
    )
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)
    result = Solve(prog)
    assert result.is_success()
    hyper_params_dict = {
        "n": n,
        "tau_limit": tau_limit,
        "initial_state": initial_state,
        "theta_limit": theta_limit,
        "speed_limit": speed_limit,
        "ladder_distance": ladder_distance,
        "final_state": final_state,
        "R": R,
        "time_panalization": time_panalization,
        "init_guess": init_guess,
        "max_timestep": max_timestep,
        "min_timestep": min_timestep,
    }
    return result, dircol, hyper_params_dict


def traj_opt_hyper(maneuver):
    '''
    n               : Number of knot points
    tau_limit       : Input torque limit
    initial_state   : Initial state: [theta1, theta2, theta1_dot, theta2_dot]
    theta_limit     : Position limits for [theta1, theta2]
    speed_limit     : Velocity limits for [theta1_dot, theta2_dot]
    ladder_distance : Distance between two ladder bars in meter
    final_state     : Final state: [theta1, theta2, theta1_dot, theta2_dot]
    R               : Input regulization term
    W               : Time panalization of the total trajectory
    init_guess      : Initial trajectory as initial guess for solver 
    '''
    hyper_dict = {
        "ZB": [
            20,
            3.0,
            (0, 0, 0, 0),
            [2.0943951023931953, 2.8797932657906435],
            10,
            0.34,
            (np.deg2rad(-32), np.deg2rad(-113), -0.5, -3),
            100,
            0,
            [0.0, 2.5],
        ],
        "ZF": [
            20,
            3.0,
            (0, 0, 0, 0),
            [2.0943951023931953, 2.8797932657906435],
            10,
            0.34,
            (np.deg2rad(39), np.deg2rad(110), -3.0, -2.5),
            100,
            50,
            [0.0, 0.5],
        ],
        "FB": [
            20,
            3.0,
            (0.506588, 2.215851, -0.634777, 4.677687),
            [2.0943951023931953, 2.8797932657906435],
            10,
            0.34,
            (np.deg2rad(-32), np.deg2rad(-113), -0.5, -3),
            100,
            0,
            [0.0, 2.0],
        ],
        "BF": [
            20,
            3.0,
            (-0.631562, -1.872709, -0.633643, 1.456502),
            [2.0943951023931953, 2.8797932657906435],
            10,
            0.34,
            (np.deg2rad(39), np.deg2rad(110), -3.0, -2.5),
            100,
            50,
            [0.0, 2.0],
        ],
    }
    return hyper_dict[f"{maneuver}"]


def create_acromonk_plant():
    from pydrake.all import MultibodyPlant, SceneGraph, Parser

    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant)
    urdf_folder = "data/urdf-files/urdf"
    file_name = "acromonk.urdf"
    up_directory = 4
    urdf_path = generate_path(urdf_folder, file_name, up_directory)
    parser.AddModelFromFile(urdf_path)
    plant.Finalize()
    context = plant.CreateDefaultContext()
    return plant, context, scene_graph


def visualize_traj_opt(plant, scene_graph, x_trajectory):
    from pydrake.all import (
        DiagramBuilder,
        MultibodyPositionToGeometryPose,
        TrajectorySource,
    )

    builder = DiagramBuilder()
    source = builder.AddSystem(TrajectorySource(x_trajectory))
    builder.AddSystem(scene_graph)
    pos_to_pose = builder.AddSystem(
        MultibodyPositionToGeometryPose(plant, input_multibody_state=True)
    )
    builder.Connect(source.get_output_port(0), pos_to_pose.get_input_port())
    builder.Connect(
        pos_to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()),
    )
    intial_state = x_trajectory.value(x_trajectory.start_time())
    drake_visualizer(
        scene_graph, builder, initial_state=intial_state ,duration=x_trajectory.end_time()
    )


def save_trajectory(
    maneuver, x_trajectory, u_trajectory, frequency, hyper_params
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
    trajectory_folder = "data/trajectories/direct_collocation"
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
    }
    save_data(data, trajectory_path)
    file_name = f"{maneuver}.txt"
    params_path = generate_path(trajectory_folder, file_name, up_directory)
    save_dict(hyper_params, params_path)
    return data


def plot_traj(traj_data):
    import matplotlib.pyplot as plt

    plt.figure(figsize=(15, 10))
    plt.plot(traj_data["time"], traj_data["shoulder_pos"], label="shoulder")
    plt.plot(traj_data["time"], traj_data["elbow_pos"], label="elbow", c='red')
    plt.title("Position")
    plt.xlabel("Time(seconds)")
    plt.ylabel("Position(rad)")
    plt.legend()

    plt.figure(figsize=(15, 10))
    plt.plot(traj_data["time"], traj_data["shoulder_vel"], label="shoulder")
    plt.plot(traj_data["time"], traj_data["elbow_vel"], label="elbow", c='red')
    plt.title("Velocity")
    plt.xlabel("Time(seconds)")
    plt.ylabel("Velocity(rad/s)")
    plt.legend()

    plt.figure(figsize=(15, 10))
    plt.plot(traj_data["time"], traj_data["elbow_torque"], label="elbow")
    plt.title("Torque Plot")
    plt.xlabel("Time(seconds)")
    plt.ylabel("Torque(Nm)")
    plt.legend()

    ee_y, ee_z = forward_kinematics(
        traj_data["shoulder_pos"], traj_data["elbow_pos"]
    )
    plt.figure(figsize=(15, 10))
    plt.plot(ee_y, ee_z, label="ee trajectory")
    plt.title("End-Effector Trajectory")
    plt.xlabel("Y-component(m)")
    plt.ylabel("Z-component(m)")
    plt.legend()

    plt.show()
