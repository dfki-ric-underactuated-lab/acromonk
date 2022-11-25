from pydrake.all import (
    PidController,
    Saturation,
    LogVectorOutput,
    TrajectorySource,
    Multiplexer, 
    Demultiplexer
)
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