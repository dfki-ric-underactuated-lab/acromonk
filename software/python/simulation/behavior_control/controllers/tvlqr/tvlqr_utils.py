from pydrake.all import (
    Saturation,
    LogVectorOutput
)
from pydrake.all import VectorSystem, PiecewisePolynomial
class TvlqrControllerSystem(VectorSystem):
    def __init__(self, plant, tvlqr):
        VectorSystem.__init__(self, 4, 1, True)
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