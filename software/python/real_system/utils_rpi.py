import os
import numpy as np
from collections import namedtuple
import moteus
import moteus_pi3hat
import time


def parent(path):
    return os.path.dirname(path)


def generate_path(path_to_folder, file_name, up_directory_counter):
    cur_dir = os.path.realpath(os.curdir)
    tmp_dir = cur_dir
    i = 0
    while i < up_directory_counter:
        tmp_dir = parent(tmp_dir)
        i += 1
    main_dir = tmp_dir
    return os.path.join(main_dir, path_to_folder, file_name)


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
        position=rad2rev(pos),
        velocity=rad2rev(vel),
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
):
    if np.sign(imu_reading) == np.sign(imu_reading):
        return imu_reading - imu_init
    else:
        return imu_reading + imu_init

async def state_estimation(pr, pos_el, vel_el):
    if abs(pos_el) > (np.pi):
        raise Exception(f"Elbow pos={pos_el} out of range [-pi, pi]")
    quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr)
    th_imu = euler_xyz[0]
    omega_x, _, _ = np.deg2rad(vel_xyz)

    th_imu_0_to_2pi = th_imu + np.pi
    th_2_0_to_2pi = pos_el + np.pi
    th_1 = np.mod(np.pi - th_2_0_to_2pi + th_imu_0_to_2pi, 2 * np.pi) - np.pi
    th_1_dot = omega_x - vel_el

    return th_1, th_1_dot, euler_xyz, np.deg2rad(vel_xyz)

def wrap_pi2pi(angle):
    # This is a function that wraps any angle in [-pi, pi]
    angle = angle % (2 * np.pi)
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle


def wrap_z2pi(angle):
    # This is a function that wraps any angle in [0, 2 * pi]
    if angle < 0:
        return angle + np.ceil(-angle / (2 * np.pi)) * 2 * np.pi
    elif angle > 2 * np.pi:
        return angle - (np.ceil(angle / (2 * np.pi)) - 1) * 2 * np.pi

    return angle


async def state_estimation_v2(pr):
    _, vel_xyz, _, euler_xyz = await read_imu_data(pr)
    theta1 = wrap_pi2pi(np.pi - euler_xyz[0])
    theta1_dot, _, _ = np.deg2rad(vel_xyz)
    return -theta1, theta1_dot, euler_xyz, np.deg2rad(vel_xyz)
    

def save_dict(dict, path):
    with open(path, "w") as f:
        for key, value in dict.items():
            f.write(f"{key}={value}\n")
    print(f"Dictonary saved in: {path}.")
    with open(path, "r") as f:
        print(f"Dictonary:\n{f.read()}")


def tau_tvlqr(
    nominal_pcws, brach_sign, des_states_array, meas_states_array, time
):
    xbar = meas_states_array - des_states_array
    K = np.array(
        [
            [
                nominal_pcws.k1.get_value(time),
                nominal_pcws.k2.get_value(time),
                nominal_pcws.k3.get_value(time),
                nominal_pcws.k4.get_value(time),
            ]
        ]
    )
    u0 = brach_sign * np.array([[nominal_pcws.el_des_tau_pcw.get_value(time)]])
    tau = u0 - np.dot(K, xbar)
    return tau, K


def forward_kinematics(theta1, theta2):
    '''
    Function to compute the forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector position given the joint angles.
    Returns ons the (y,z) coordinates as the AcroMonk can only move in a plane.
    '''
    l1 = 0.31401
    l2 = l1    
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta2))
    ee_z = -(l1 * np.cos(theta1) + (l2 * np.cos(theta1 + theta2)))

    return ee_y, ee_z


def forward_diff_kinematics(theta1, theta2, theta1_dot, theta2_dot):
    '''
    Function to compute the differential forward kinematics of the AcroMonk Robot,
    i.e. compute the end-effector velocity given the join position/velocities
    Returns ons the (y_dot,z_dot) coordinates as the AcroMonk can only move in a plane.
    '''
    l1 = 0.31401
    l2 = l1    
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (l2 * (theta1_dot + theta2_dot) * np.cos(theta1 + theta2))
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) -(l2 * (theta1_dot + theta2_dot) * np.sin(theta1 + theta2))
    return ee_y_dot, ee_z_dot


def calc_pd_tau(
    des_pos,
    meas_pos,
    des_vel,
    meas_vel,
    gear_ratio
    ):    
    Kp = 100
    Kd = 2
    vel_error = des_vel - meas_vel
    pos_error = des_pos - meas_pos
    tau_pd = Kp * (pos_error) + Kd * (vel_error)
    tau_pd /= gear_ratio
    return tau_pd 


def motor_test_parameters(
    test, nominal_pcws, brach_sign, meas_state, des_state, time, controller
):
    if test == "pd":
        kp_scale = 1
        kd_scale = 1
        tau_cmd = 0
    elif test == "ff_replay":
        u_ff = nominal_pcws.el_tau_tvlqr_pcw.get_value(time)
        kp_scale = 0
        kd_scale = 0
        tau_cmd = u_ff
    elif test == "tvlqr":
        u_tvlqr, K = tau_tvlqr(
            nominal_pcws=nominal_pcws,
            brach_sign=brach_sign,
            des_states_array=des_state,
            meas_states_array=meas_state,
            time=time,
        )
        kp_scale = 0
        kd_scale = 0
        tau_cmd = u_tvlqr
    elif test == "rl":
        # flip elbow states for el controller in even brachiations
        if brach_sign == -1:
            meas_state = np.multiply(meas_state, [[1], [-1], [1], [-1]])
        u_rl = controller.get_torque(meas_state)
        kp_scale = 0
        kd_scale = 0
        tau_cmd = u_rl
    else:
        raise Exception("Wrong test input!")
    Data = namedtuple("Data", ["kp_scale", "kd_scale", "tau_cmd", "K_tvlqr"])
    data = Data(
        kp_scale=kp_scale,
        kd_scale=kd_scale,
        tau_cmd=tau_cmd,
        K_tvlqr=K if test == "tvlqr" else np.zeros((1, 4)),
    )
    return data


async def go_to_pos(servo, final_pos, rad_per_cycle=0.003, tau_limit = 4):

    print('[go_to_pos]')
    await servo.set_stop()    
    (pos, _, _) = await read_motor_data(servo)    
    cmd_pos = pos
    sgn = -1 if pos > final_pos else 1
    try:
        while sgn * (final_pos - pos) > 0:
            cmd_pos += sgn * rad_per_cycle
            (pos,
            vel,
            tau) = await send_rad_command(
                controller_obj=servo,
                pos=cmd_pos,
                vel=0.0,
                tau_limit=tau_limit,
                watchdog_timeout=0.1)
    finally:
        await servo.set_stop()


async def arms_attached(imu, servo, number_of_loops, brach_type):
    print('[arms_attached]')
    temp_array_ee_z=np.zeros(number_of_loops)
    temp_array_ee_y=np.zeros(number_of_loops)
    while True:
        _, omegas, _, _ = await read_imu_data(imu)
        omegas = abs(np.deg2rad(omegas))
        if all((omegas) <= 0.05):
            break
        time.sleep(0.3)
    index = 0
    while index < number_of_loops:
        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,
            vel=0.0,
            tau=0.0,
            tau_limit=1.,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )        
        if brach_type == 'odd':
            state = await state_estimation(
                    pr=imu,
                    pos_el=el_pos,
                    vel_el=el_vel
                )
        else:        
            state = await state_estimation_v2(imu)    
        (sh_pos, sh_vel, raw_imu_pos, raw_imu_vel) = state                            
        roll, pitch, yaw = raw_imu_pos          
        (ee_y, ee_z) = forward_kinematics(abs(sh_pos), abs(el_pos))
        if abs(pitch) >= 0.35 or abs(yaw) >= 0.35 or abs(ee_y) >0.5: 
            return 0 
        temp_array_ee_z[index] = ee_z
        temp_array_ee_y[index] = ee_y
        index += 1  
    conf = None
    if (abs(roll) < np.deg2rad(90.0)) or (roll > np.deg2rad(-170.0) and roll < np.deg2rad(-90.0)):
        conf = 1
    elif (abs(roll) >= np.deg2rad(170.0)) or (roll > np.deg2rad(90.0) and roll < np.deg2rad(170.0)):
        conf = 2 
    if (np.mean(abs(temp_array_ee_z))) <= 0.02 and (np.mean(abs(temp_array_ee_y))) > 0.32 and (np.mean(abs(temp_array_ee_y))) < 0.4:
        print('[arms_attached]ended')
        return 2, conf
    else:
        print('[arms_attached]ended')
        return 1, conf
