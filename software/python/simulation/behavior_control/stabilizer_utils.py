import sys
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
