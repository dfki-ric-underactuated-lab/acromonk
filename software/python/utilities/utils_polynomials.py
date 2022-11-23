import numpy as np
from scipy.optimize import curve_fit as cf
from pydrake.all import PiecewisePolynomial
from collections import namedtuple


class FitPiecewisePolynomial:
    """
    Gets data and number of break points and
    fit cubic segment polynomials to each section of data
    """

    def __init__(self, data_x, data_y, num_break, poly_degree):
        self.data_x = data_x
        self.data_y = data_y
        self.num_break = num_break
        self.poly_degree = poly_degree
        (
            self.x_sec_data,
            self.y_sec_data,
            self.coeff_sec_data,
            self.t0_polys,
            self.tf_polys,
        ) = self.create_section_poly()

    def determin_poly(self):
        if self.poly_degree == 1:
            return self.poly1
        elif self.poly_degree == 2:
            return self.poly2
        elif self.poly_degree == 3:
            return self.poly3
        else:
            print('Choose between "1,2,3" for the degree of the polynomial')
            return None

    def poly1(self, t, A, B):
        return A * pow(t, 1) + B

    def poly2(self, t, A, B, C):
        return A * pow(t, 2) + B * pow(t, 1) + C

    def poly3(self, t, A, B, C, D):
        return A * pow(t, 3) + B * pow(t, 2) + C * pow(t, 1) + D

    def end_time(self):
        return self.data_x[-1]

    def start_time(self):
        return self.data_x[0]

    def split_data(self, data):
        """
        Takes the original data and return a list of splitted arrays
        """
        l = len(data)
        sec_len = int(np.ceil(l / self.num_break))
        return np.array_split(data, self.num_break)

    def create_section_poly(self):
        """
        This function takes the splitted data(x, y) and return 2 lists
        - list of the x-data to be fitted to the setion data
        - list of the fitted value
        """
        splitted_data_x = self.split_data(self.data_x)
        splitted_data_y = self.split_data(self.data_y)
        x_sec_data = []
        y_sec_data = []
        coeff_sec_data = []
        index = 0
        for sec in splitted_data_x:
            x_sec_data.append(np.linspace(sec[0], sec[-1], len(sec)))
            func = self.determin_poly()
            p_coeff, p_cov = cf(
                func, splitted_data_x[index], splitted_data_y[index]
            )
            fit = func(x_sec_data[index], *p_coeff)
            y_sec_data.append(fit)
            coeff_sec_data.append(p_coeff)
            index += 1
        t0_polys = np.asarray([sec_t[0] for sec_t in x_sec_data])
        tf_polys = np.asarray([sec_t[-1] for sec_t in x_sec_data])
        self.x_sec_data = x_sec_data
        self.y_sec_data = y_sec_data
        self.coeff_sec_data = coeff_sec_data
        self.t0_polys = t0_polys
        self.tf_polys = tf_polys
        return x_sec_data, y_sec_data, coeff_sec_data, t0_polys, tf_polys

    def get_value(self, time_stamp):

        if time_stamp > self.data_x[-1]:
            print(f"time exceeded,time={time_stamp}, value={self.data_y[-1]}")
            return self.data_y[-1]
        else:
            poly_index = len(
                np.where(np.array(time_stamp <= self.tf_polys) == False)[0]
            )
            p_coeff = self.coeff_sec_data[poly_index]
            func = self.determin_poly()
            return func(time_stamp, *p_coeff)


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