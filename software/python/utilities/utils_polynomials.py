import numpy as np
from pydrake.all import PiecewisePolynomial
from collections import namedtuple
from utils import save_data, save_dict, make_parent_directory


def extract_data_from_polynomial(polynomial, frequency):
    n_points = int(polynomial.end_time() / (1 / frequency))
    time_traj = np.linspace(
        polynomial.start_time(), polynomial.end_time(), n_points
    )
    # extracted_time = time_traj.reshape(n_points, 1).T
    extracted_data = np.hstack(
        [
            polynomial.value(t)
            for t in np.linspace(
                polynomial.start_time(), polynomial.end_time(), n_points
            )
        ]
    )
    return extracted_data, time_traj


def create_gain_arrays(tvlqr_obj, frequency):
    n_points = int(tvlqr_obj.K.end_time() / (1 / frequency))
    time_stamp = np.linspace(
        tvlqr_obj.K.start_time(), tvlqr_obj.K.end_time(), n_points
    )
    K = [tvlqr_obj.K.value(i) for i in time_stamp]
    index = 0
    K_array = np.zeros((n_points, 4))
    for k_vec in K:
        K_array[index] = k_vec[0]
        index += 1
    k0 = np.array([tvlqr_obj.k0.value(i)[0][0] for i in time_stamp])
    return K_array, k0, time_stamp


def prepare_des_states(csv_data):
    ## shoulder
    time = csv_data["time"].values
    shoulder_des_pos = csv_data["shoulder_pos"].values
    shoulder_des_vel = csv_data["shoulder_vel"].values
    ## elbow
    elbow_des_pos = csv_data["elbow_pos"].values
    elbow_des_vel = csv_data["elbow_vel"].values
    elbow_des_tau = csv_data["elbow_torque"].values
    Data = namedtuple(
        "Data",
        [
            "time",
            "shoulder_des_pos",
            "shoulder_des_vel",
            "elbow_des_pos",
            "elbow_des_vel",
            "elbow_des_tau"
        ],
    )
    data = Data(
        shoulder_des_pos=shoulder_des_pos,
        shoulder_des_vel=shoulder_des_vel,
        elbow_des_pos=elbow_des_pos,
        elbow_des_vel=elbow_des_vel,
        elbow_des_tau=elbow_des_tau,
        time=time,
    )
    return data


def fit_polynomial(data):
    """
    This function takes a data as input and fit a polynomial of degree 1 to the torque and
    a cubic one to state, and derivative of order 1 and 2 of states and returns the polynomials
    """
    csv_data = prepare_des_states(data)
    elbow_des_tau = csv_data.elbow_des_tau.reshape(
        csv_data.elbow_des_tau.shape[0], -1
    ).T
    des_time = csv_data.time.reshape(csv_data.time.shape[0], -1)
    x0_desc = np.vstack(
        (
            csv_data.shoulder_des_pos,
            csv_data.elbow_des_pos,
            csv_data.shoulder_des_vel,
            csv_data.elbow_des_vel,
        )
    )
    u0_desc = elbow_des_tau
    u0 = PiecewisePolynomial.FirstOrderHold(des_time, u0_desc)
    x0 = PiecewisePolynomial.CubicShapePreserving(
        des_time, x0_desc, zero_end_point_derivatives=True
    )
    x0_d = x0.derivative(derivative_order=1)
    x0_dd = x0.derivative(derivative_order=2)
    return x0, u0, x0_d, x0_dd