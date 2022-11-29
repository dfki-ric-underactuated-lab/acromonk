import time
import os
import numpy as np
import math
import moteus
import moteus_pi3hat


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def point_in_hull(point, hull, tolerance=1e-12):
    return all((np.dot(eq[:-1], point) + eq[-1] <= tolerance) for eq in hull.equations)


def rad2rev(angle_in_radians):
    return angle_in_radians * (1 / (2 * np.pi))


def rev2rad(angle_in_revolution):
    return angle_in_revolution * (2 * np.pi)


def zero_offset(bus_number, motor_id):
    os.system(
        f"sudo moteus_tool --zero-offset  --pi3hat-cfg '{bus_number}={motor_id}' -t {motor_id}"
    )
    print(f"motor {motor_id} zero offset was successful.")


async def send_rad_command(
    controller_obj=None,
    pos=None,
    vel=None,
    tau=None,
    tau_limit=None,
    kp_scale=1,
    kd_scale=1,
    watchdog_timeout=None,
):
    state = await controller_obj.set_position(
        position=rad2rev(pos),  # 0,#
        velocity=rad2rev(vel),  # 0,#
        kp_scale=kp_scale,
        kd_scale=kd_scale,
        stop_position=None,
        feedforward_torque=tau,
        maximum_torque=tau_limit,
        watchdog_timeout=watchdog_timeout,
        query=True,
    )
    # store data
    meas_pos = rev2rad(state.values[moteus.Register.POSITION])
    meas_vel = rev2rad(state.values[moteus.Register.VELOCITY])
    meas_tau = state.values[moteus.Register.TORQUE]
    return meas_pos, meas_vel, meas_tau


async def read_motor_data(controller_obj=None):
    state = await controller_obj.set_position(
        kp_scale=0, kd_scale=0, query=True
    )
    meas_pos = rev2rad(state.values[moteus.Register.POSITION])
    meas_vel = rev2rad(state.values[moteus.Register.VELOCITY])
    meas_tau = state.values[moteus.Register.TORQUE]
    return meas_pos, meas_vel, meas_tau


async def read_imu_data(pr):
    imu = await pr.cycle([], request_attitude=True)
    imu_data = imu[0]
    quat_wxyz = (
        imu_data.attitude.w,
        imu_data.attitude.x,
        imu_data.attitude.y,
        imu_data.attitude.z,
    )
    vel_xyz = imu_data.rate_dps.x, imu_data.rate_dps.y, imu_data.rate_dps.z
    acc_xyz = (
        imu_data.accel_mps2.x,
        imu_data.accel_mps2.y,
        imu_data.accel_mps2.z,
    )
    euler_xyz = (
        imu_data.euler_rad.roll,
        imu_data.euler_rad.pitch,
        imu_data.euler_rad.yaw,
    )
    return quat_wxyz, vel_xyz, acc_xyz, euler_xyz


def imu_zero_offset(
    imu_reading, imu_init
):  # FIXME This creates bugs and wrapper problem
    if np.sign(imu_reading) == np.sign(imu_reading):
        return imu_reading - imu_init
    else:
        return imu_reading + imu_init


def quat2angle(w, x, y, z):
    denom = math.sqrt(1 - w * w)
    ax = x / denom
    ay = y / denom
    az = z / denom
    angle_axis = 2 * math.acos(w)
    return (angle_axis, ax, ay, az, denom)


def quaternion_to_euler(w, x, y, z):
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.asin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    Z = math.atan2(t3, t4)
    return X, Y, Z


def quat_magnitude(q):
    w, x, y, z = q
    return (w ** 2 + x ** 2 + y ** 2 + z ** 2) ** 0.5


def quat_dot(q1, q2):
    q1q2 = tuple(l * r for l, r in zip(q1, q2))
    return q1q2


def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


# def qp_mult(q1, p1):
#     q2 = (0.0,) + p1
#     return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]


def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


def quaternion_to_euler(quat_tuple):
    w, x, y, z = quat_tuple
    t0 = 2 * (w * x + y * z)
    t1 = 1 - 2 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = 2 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.asin(t2)

    t3 = 2 * (w * z + x * y)
    t4 = 1 - 2 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


def quat_magnitude(q):
    w, x, y, z = q
    return (w ** 2 + x ** 2 + y ** 2 + z ** 2) ** 0.5


def quat_dot(q1, q2):
    q1q2 = tuple(l * r for l, r in zip(q1, q2))
    return q1q2


def get_angle(q1, q2):
    q1q2 = quat_dot(q1, q2)
    return 2 * math.acos(sum(q1q2) / (quat_magnitude(q1) * quat_magnitude(q2)))


def get_angle_atan2(q1, q2):
    q1 = np.asarray(q1)
    q2 = np.asarray(q2)
    cos_theta = q1.dot(q2) / (np.linalg.norm(q1) * np.linalg.norm(q2))
    sin_theta = math.sqrt(1 - cos_theta ** 2)
    return 2 * math.atan2(sin_theta, cos_theta)


def get_empirical_roa_points():
    roa_points_array = np.array([[0.73659, 1.98063],
                                 [0.71039, 1.95483],
                                 [0.68159, 1.91543],
                                 [0.62469, 2.00303],
                                 [0.7421, 1.9726],
                                 [0.7159, 1.9468],
                                 [0.6871, 1.9074],
                                 [0.6302, 1.995],
                                 [0.7321, 1.9824],
                                 [0.7059, 1.9566],
                                 [0.6771, 1.9172],
                                 [0.6202, 2.0048],
                                 [0.7464, 1.9716],
                                 [0.7202, 1.9458],
                                 [0.6914, 1.9064],
                                 [0.6345, 1.994],
                                 [0.7411, 1.9778],
                                 [0.7149, 1.952],
                                 [0.6861, 1.9126],
                                 [0.6292, 2.0002],
                                 [0.7287, 2.0006],
                                 [0.7025, 1.9748],
                                 [0.6737, 1.9354],
                                 [0.6168, 2.023],
                                 [0.7357, 1.9752],
                                 [0.7095, 1.9494],
                                 [0.6807, 1.91],
                                 [0.6238, 1.9976],
                                 [0.7316, 1.9849],
                                 [0.7054, 1.9591],
                                 [0.6766, 1.9197],
                                 [0.6197, 2.0073],
                                 [0.7396, 1.9759],
                                 [0.7134, 1.9501],
                                 [0.6846, 1.9107],
                                 [0.6277, 1.9983],
                                 [0.7406, 1.9766],
                                 [0.7144, 1.9508],
                                 [0.6856, 1.9114],
                                 [0.6287, 1.999],
                                 [0.728, 1.9887],
                                 [0.7018, 1.9629],
                                 [0.673, 1.9235],
                                 [0.6161, 2.0111]])

    return roa_points_array


def create_imu_object(config_in_degree, convergence_delay):
    imu = moteus_pi3hat.Pi3HatRouter(
        mounting_deg={
            "roll": config_in_degree[0],
            "pitch": config_in_degree[1],
            "yaw": config_in_degree[2]}
    )
    time.sleep(convergence_delay)
    return imu


def create_servo_object(bus_number, motor_id):
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            bus_number: [motor_id],
        },
    )
    servo = moteus.Controller(id=motor_id, transport=transport)
    return servo

