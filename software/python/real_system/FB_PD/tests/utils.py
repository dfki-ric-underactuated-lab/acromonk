try:
    import matplotlib.pyplot as plt
    import os
    import pandas as pd
    import numpy as np
    import math
    from datetime import datetime
    import matplotlib.transforms as mtransforms
    import asyncio
    from scipy.optimize import curve_fit as cf 
    from collections import namedtuple
    import moteus
    import moteus_pi3hat
except BaseException as e:
    print(e)    
try:        
    from pydrake.all import (DirectCollocation,
                            PiecewisePolynomial,
                            Solve,
                            Parser,
                            MultibodyPlant,
                            SceneGraph,
                            DiagramBuilder,
                            MultibodyPositionToGeometryPose,
                            TrajectorySource,
                            ConnectMeshcatVisualizer,
                            Simulator)
except BaseException as e:
    print(e)    


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
    
    
def read_data(folder, file, up_directory):
    path_to_file = generate_path(folder, file, up_directory)
    data = pd.read_csv(path_to_file)
    return data


def plot_data(x_data, 
              y_data, 
              labels, 
              title, 
              legends=None, 
              save_name=None,
              linestyle=None, 
              linewidth=None,
              colors=None):
    plt.figure(figsize=(15,10))
    if linestyle is None:
        linestyle = [None] * len(x_data)
    if colors is None:
        colors = [None] * len(x_data)        
    for i in range(len(x_data)):
        plt.plot(x_data[i], 
                 y_data[i],
                 linestyle=linestyle[i],
                 linewidth=linewidth,
                 c=colors[i])
    
    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.title(title)
    if legends is None:
        pass
    else:
        plt.legend(legends)
    plt.draw()
    if save_name is None:
        pass
    else:    
        print(f"Making {save_name} plot.") 
        plt.savefig(save_name)
    return plt
        

def prepare_data(csv_data):

    n = len(csv_data)
    # empty arrays for measured data
    (shoulder_meas_pos,
     shoulder_meas_vel,
     _,
     _,
     elbow_meas_pos,
     elbow_meas_vel,
     elbow_meas_tau,
     _,
     meas_time) = prepare_empty_arrays(n)
    # desired trajectory data
    ## shoulder
    des_time = csv_data["time"].values
    shoulder_des_pos = csv_data["shoulder_pos"].values
    shoulder_des_vel = csv_data["shoulder_vel"].values
    shoulder_des_acc = csv_data["shoulder_acc"].values
    shoulder_des_jerk = csv_data["shoulder_jerk"].values
    shoulder_des_tau = csv_data["shoulder_torque"].values
    ## elbow.values
    elbow_des_pos = csv_data["elbow_pos"].values
    elbow_des_vel = csv_data["elbow_vel"].values
    elbow_des_acc = csv_data["elbow_acc"].values
    elbow_des_jerk = csv_data["elbow_jerk"].values
    elbow_des_tau = csv_data["elbow_torque"].values
    elbow_des_tau_tvlqr = csv_data["elbow_torque_tvlqr"].values
    # K values
    k1 = csv_data["K1"].values
    k2 = csv_data["K2"].values
    k3 = csv_data["K3"].values
    k4 = csv_data["K4"].values
    k0 = csv_data["k0"].values

    # converting the desired trajectories according to the gear ratio
    dt = csv_data["time"][2] - csv_data["time"][1]
    Data=namedtuple('Data',
            ['shoulder_des_pos',
             'shoulder_des_vel',
             'shoulder_des_acc',
             'shoulder_des_jerk',
             'shoulder_des_tau',
             'elbow_des_pos',
             'elbow_des_vel',
             'elbow_des_acc',
             'elbow_des_jerk',
             'elbow_des_tau',
             'elbow_des_tau_tvlqr',
             'des_time',
             'shoulder_meas_pos',
             'shoulder_meas_vel',
             'elbow_meas_pos',
             'elbow_meas_vel',
             'elbow_meas_tau',
             'meas_time',             
             'k1',
             'k2',
             'k3',
             'k4',
             'k0',             
             'n',
             'dt']) 
    data = Data(
                shoulder_des_pos=shoulder_des_pos,
                shoulder_des_vel=shoulder_des_vel,
                shoulder_des_acc=shoulder_des_acc,
                shoulder_des_jerk=shoulder_des_jerk,
                shoulder_des_tau=shoulder_des_tau,
                elbow_des_pos=elbow_des_pos,
                elbow_des_vel=elbow_des_vel,
                elbow_des_acc=elbow_des_acc,
                elbow_des_jerk=elbow_des_jerk,
                elbow_des_tau=elbow_des_tau,
                elbow_des_tau_tvlqr=elbow_des_tau_tvlqr,
                des_time=des_time,
                shoulder_meas_pos=shoulder_meas_pos,
                shoulder_meas_vel=shoulder_meas_vel,
                elbow_meas_pos=elbow_meas_pos,
                elbow_meas_vel=elbow_meas_vel,
                elbow_meas_tau=elbow_meas_tau,
                meas_time=meas_time,
                k1=k1,
                k2=k2,
                k3=k3,
                k4=k4,
                k0=k0,
                n=n,
                dt=dt,
                )
    return data


def prepare_empty_arrays(n):
    shoulder_meas_pos = np.zeros(n)
    shoulder_meas_vel = np.zeros(n)
    shoulder_meas_tau = np.zeros(n)
    shoulder_cmd_tau = np.zeros(n)
    elbow_meas_pos = np.zeros(n)
    elbow_meas_vel = np.zeros(n)
    elbow_meas_tau = np.zeros(n)
    elbow_cmd_tau = np.zeros(n)
    meas_time = np.zeros(n)
    return (shoulder_meas_pos,
            shoulder_meas_vel,
            shoulder_meas_tau,
            shoulder_cmd_tau,
            elbow_meas_pos,
            elbow_meas_vel,
            elbow_meas_tau,
            elbow_cmd_tau,
            meas_time)
    
    
def save_data(data, path_to_save):
    header = ','.join(data.keys())
    data = np.vstack(data.values()).T
    np.savetxt(path_to_save,
               data, delimiter=',',
               header=header,
               comments="")
    print(f'saved csv to {path_to_save}')


def rad2rev(angle_in_radians):
    return angle_in_radians * (1 / (2 * np.pi))


def rev2rad(angle_in_revolution):
    return angle_in_revolution * (2 * np.pi)


def zero_offset(bus_number, motor_id):
    os.system(f"sudo moteus_tool --zero-offset  --pi3hat-cfg '{bus_number}={motor_id}' -t {motor_id}")
    print(f'motor {motor_id} zero offset was successful.')         


async def send_rad_command(controller_obj=None,
                       pos=None,
                       vel=None,
                       tau=None,
                       tau_limit=None,
                       kp_scale=1,
                       kd_scale=1,
                       watchdog_timeout=None):
        state = await controller_obj.set_position(
                                    position=rad2rev(pos),  # 0,#
                                    velocity=rad2rev(vel),  # 0,#
                                    kp_scale=kp_scale,
                                    kd_scale=kd_scale,
                                    stop_position=None,
                                    feedforward_torque=tau,
                                    maximum_torque=tau_limit,
                                    watchdog_timeout = watchdog_timeout,
                                    query=True)
        # store data
        meas_pos = rev2rad(state.values[moteus.Register.POSITION]) 
        meas_vel = rev2rad(state.values[moteus.Register.VELOCITY])
        meas_tau = state.values[moteus.Register.TORQUE]
        return meas_pos, meas_vel, meas_tau


async def read_motor_data(controller_obj = None):
    state = await controller_obj.set_position(
        kp_scale=0,
        kd_scale=0,
        query=True)    
    meas_pos = rev2rad(state.values[moteus.Register.POSITION]) 
    meas_vel = rev2rad(state.values[moteus.Register.VELOCITY])
    meas_tau = state.values[moteus.Register.TORQUE]
    return meas_pos, meas_vel, meas_tau        

    
async def read_imu_data(pr):
    imu = await pr.cycle([], request_attitude=True)   
    imu_data = imu[0]  
    quat_wxyz = imu_data.attitude.w, imu_data.attitude.x, imu_data.attitude.y, imu_data.attitude.z                
    vel_xyz = imu_data.rate_dps.x, imu_data.rate_dps.y, imu_data.rate_dps.z        
    acc_xyz = imu_data.accel_mps2.x, imu_data.accel_mps2.y, imu_data.accel_mps2.z
    euler_xyz = imu_data.euler_rad.roll, imu_data.euler_rad.pitch, imu_data.euler_rad.yaw
    return quat_wxyz, vel_xyz, acc_xyz, euler_xyz


def imu_zero_offset(imu_reading, imu_init):
    if np.sign(imu_reading) == np.sign(imu_reading):
        return imu_reading - imu_init
    else:
        return imu_reading + imu_init


def determin_sign(roll, ax):
    sign = int(np.sign(roll))
    if int(np.sign(roll)) * int(np.sign(ax)) < 0 :
        sign = int(np.sign(ax))
    return sign    


def quat2angle(w,x,y,z):
    denom = math.sqrt(1-w*w)
    ax = x / denom
    ay = y / denom
    az = z / denom  
    angle_axis = 2 * math.acos(w)
    return (angle_axis, ax, ay, az, denom)   


async def state_estimation(pr, pos_el, vel_el, imu_init):
    if abs(pos_el) > (np.pi):
        raise Exception(f"Elbow pos={pos_el} out of range [-2pi,2pi]")
    quat_wxyz, vel_xyz, acc_xyz, euler_xyz = await read_imu_data(pr)
    w, x, y, z = quat_wxyz
    omega_x, _, _ = np.deg2rad(vel_xyz)
    _,ax,_,_,_ = quat2angle(w, x, y, z)
    raw_imu = euler_xyz[0]    
    sign = determin_sign(raw_imu, ax)
    if sign > 0:
        es_theta1 = wrap_z2pi(raw_imu) - pos_el
    elif sign == 0:
        es_theta1 = 0
    elif sign < 0:
        es_theta1 = (- 2 * np.pi + wrap_z2pi(raw_imu)) - pos_el  
    if abs(es_theta1) > np.deg2rad(358):
        es_theta1 = 0


    if abs(imu_init) < np.deg2rad(5):
        es_theta1 = imu_zero_offset(es_theta1, imu_init)        
    es_theta1_dot = omega_x - vel_el
    return es_theta1, es_theta1_dot, raw_imu, omega_x   


async def state_estimation_v2(pr):
    _, vel_xyz, _, euler_xyz = await read_imu_data(pr)
    theta1 = euler_xyz[0]
    theta1_dot, _, _ = np.deg2rad(vel_xyz)
    return theta1, theta1_dot
      
      
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


def print_val(**kwargs):
    for name, value in kwargs.items():
        print("%s = \n%s" % (name, repr(value)))


def get_theta1(imu_angle, theta2):

    return wrap_pi2pi(wrap_z2pi(imu_angle - theta2))


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
    w,x,y,z = q
    return (w**2 + x**2 + y**2 + z**2)** 0.5


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
    w,x,y,z = q
    return (w**2 + x**2 + y**2 + z**2)** 0.5


def quat_dot(q1, q2):
    q1q2 = tuple(l * r for l, r in zip(q1, q2))
    return q1q2    


def get_angle(q1, q2):
    q1q2 = quat_dot(q1,q2)
    return 2 * math.acos(sum(q1q2)/(quat_magnitude(q1) * quat_magnitude(q2)))
        

def get_angle_atan2(q1, q2):
    q1 = np.asarray(q1)
    q2 = np.asarray(q2)
    cos_theta = (q1.dot(q2)/(np.linalg.norm(q1) * np.linalg.norm(q2)))        
    sin_theta = math.sqrt(1 - cos_theta**2)
    return 2*math.atan2(sin_theta, cos_theta)
    
    
def plot_analysis_data(data, laptop_timeout, rpi_timeout):
        date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
        flags = data["flag"]
        waitings = np.reshape(np.where(flags==1), -1)
        fig, ax = plt.subplots(nrows=1,ncols=2,figsize=(15, 10))
        ax[0].plot(data["time"], data["meas_pos"], label='meas')
        ax[0].plot(data["time"], data["des_pos"], label='des')
        ax[0].legend()
        ax[1].plot(data["time"], data["latency"],label='latency')
        ax[1].legend()
        #trans = mtransforms.blended_transform_factory(ax.transData, ax.transAxes)
        # ax[0].fill_between(data["time"],max(abs(data["des_pos"])) + 0.5, where=data["flag"]>0,
        #                 facecolor='red', alpha=0.5, transform=ax[0].get_xaxis_transform())      
        ax[0].set_xlabel('time')
        ax[0].set_ylabel('position (rad)')
        ax[1].set_xlabel('time')
        ax[1].set_ylabel('latency')        
        ax[0].set_title(f'Position Plot: laptop_timeout= {laptop_timeout}, rpi_timeout = {rpi_timeout}')
        for i in waitings:
            for j in range(2):
                    ax[j].axvline(data["time"][i], linestyle = ':', color='indianred')          
        fig.savefig('timeout_results/'+ date + f'_pos_lp_{laptop_timeout}_rpi_{rpi_timeout}.pdf')


        fig, axs = plt.subplots(nrows=2, ncols=2, figsize=(15, 10))
        plt.suptitle(f'Laptop timeout = {laptop_timeout}/RPI timeout = {rpi_timeout}')

        axs[0, 0].set_title("main cycle")
        axs[0, 0].scatter(data["cmd_id"], 
                          data["rpi_main_cycle_dt"], 
                          s=8 )#,
                        #   label=f'rpi: mean={np.mean(data["rpi_main_cycle_dt"]):.6f} s= {round(1/np.mean(data["rpi_main_cycle_dt"]))} Hz')
        axs[0, 0].scatter(data["cmd_id"], 
                          data["laptop_main_cycle_dt"], 
                          s=8 )#,
                        #   label=f'laptop: mean={np.mean(data["laptop_main_cycle_dt"]):.6f} s= {round(1/np.mean(data["laptop_main_cycle_dt"]))} Hz')                          
        axs[0, 0].set_ylabel("dt")
        axs[0, 0].set_xlabel("cmd_id")
        # axs[0, 0].legend()

        axs[0, 1].set_title("subscription")
        axs[0, 1].scatter(data["cmd_id"], 
                          data["rpi_sub_dt"], 
                          s=8)#, 
                        #   label=f'rpi: mean={np.mean(data["rpi_sub_dt"]):.6f} s')
        axs[0, 1].scatter(data["cmd_id"],
                          data["laptop_sub_dt"], 
                          s=1)#, 
                        #   label=f'laptop: mean={np.mean(data["laptop_sub_dt"]):.6f} s')
        axs[0, 1].set_ylabel("dt")
        axs[0, 1].set_xlabel("cmd_id")               
        # axs[0, 1].legend()
        
        axs[1, 0].set_title("publish")
        axs[1, 0].scatter(data["cmd_id"], 
                          data["rpi_pub_dt"], 
                          s=8)#, 
                        #   label=f'rpi: mean={np.mean(data["rpi_pub_dt"]):.6f} s')
        axs[1, 0].scatter(data["cmd_id"], 
                          data["laptop_pub_dt"], 
                          s=1)#, 
                          #label=f'laptop: mean={np.mean(data["laptop_pub_dt"]):.6f} s')        
        axs[1, 0].set_ylabel("dt")
        axs[1, 0].set_xlabel("cmd_id")        
        # axs[1, 0].legend()
        axs[1, 1].set_title("rpi execution")
        axs[1, 1].scatter(data["cmd_id"], 
                          data["rpi_exec_dt"], 
                          s=8)#, 
                        #   label=f'rpi: mean={np.mean(data["rpi_exec_dt"]):.6f} s')
        axs[1, 1].set_ylabel("dt")
        axs[1, 1].set_xlabel("cmd_id")        
        # axs[1, 1].legend()        
        fig.tight_layout()      
        for i in waitings:
            for j in range(2):
                for k in range(2):
                    axs[j,k].axvline(data["cmd_id"][i], linestyle = ':', color='indianred')        
        fig.savefig('timeout_results/'+ date + f'_analysis_p_{laptop_timeout}_rpi_{rpi_timeout}.pdf')                                                        

        plt.show()    


def fit_polynomial(data):
    '''
    This function takes a data as input and fit a polynomial of degree 1 to the torque and 
    a cubic one to state, and derivative of order 1 and 2 of states and returns the polynomials
    '''
    csv_data = prepare_data(data)
    csv_data.shoulder_des_pos    
    elbow_des_tau = csv_data.elbow_des_tau.reshape(csv_data.elbow_des_tau.shape[0], -1).T
    des_time = csv_data.des_time.reshape(csv_data.des_time.shape[0], -1)
    x0_desc = np.vstack((csv_data.shoulder_des_pos,
                         csv_data.elbow_des_pos,
                         csv_data.shoulder_des_vel,
                         csv_data.elbow_des_vel))
    u0_desc = elbow_des_tau
    u0 = PiecewisePolynomial.FirstOrderHold(des_time, u0_desc)
    x0 = PiecewisePolynomial.CubicShapePreserving(des_time, x0_desc, zero_end_point_derivatives=True)
    x0_d = x0.derivative(derivative_order=1)
    x0_dd = x0.derivative(derivative_order=2)
    return x0, u0, x0_d, x0_dd


def extract_data_from_polynomial(polynomial, frequency):
    n_points = int(polynomial.end_time() / (1 / frequency))
    time_traj = np.linspace(polynomial.start_time(),
                            polynomial.end_time(),
                            n_points)
    extracted_time = time_traj.reshape(n_points, 1).T
    extracted_data = np.hstack([polynomial.value(t) for t in
                                np.linspace(polynomial.start_time(),
                                            polynomial.end_time(),
                                            n_points)])
    return extracted_data, extracted_time    


def ff_acromonk(theta1, theta2, theta1_dot, theta2_dot):
    l1 = 0.31401#0.32
    l2 = l1
    ee_y = (l1 * np.sin(theta1)) + (l2 * np.sin(theta1 + theta2))
    ee_y_dot = (l1 * theta1_dot * np.cos(theta1)) + (l2 * (theta1_dot + theta2_dot) * np.cos(theta1 + theta2))
    ee_z =   (-l1 * np.cos(theta1) - (l2 * np.cos(theta1 + theta2)))
    ee_z_dot = (l1 * theta1_dot * np.sin(theta1)) -(l2 * (theta1_dot + theta2_dot) * np.sin(theta1 + theta2))
    return ee_y, ee_z, ee_y_dot, ee_z_dot


def save_dict(dict, path):
    with open(path,'w') as f:
        for key,value in dict.items():
            f.write(f'{key}={value}\n')
    print(f'Dictonary saved in: {path}.')
    with open(path,'r') as f:  
        print(f'Dictonary:\n{f.read()}')


def trajopt(plant,
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
            init_guess):
    min_timestep = 0.1
    max_timestep = 0.8        
    dircol = DirectCollocation(plant,
                           context,
                           num_time_samples=n,
                           minimum_timestep=min_timestep,\
                           maximum_timestep=max_timestep,
                           input_port_index = plant.get_actuation_input_port().get_index())
    dircol.AddEqualTimeIntervalsConstraints()
    # Add input limits.
    torque_limit = tau_limit # N*m.
    u_init = dircol.input(0)
    dircol.AddConstraintToAllKnotPoints(u_init[0] == 0)    
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(-torque_limit <= u[0])
    dircol.AddConstraintToAllKnotPoints(u[0] <= torque_limit)
    initial_state = initial_state
    
    #testing bounding box non existing attribute error
    prog = dircol.prog()
    
    prog.AddBoundingBoxConstraint(initial_state, 
                                    initial_state,
                                    dircol.initial_state())
    state = dircol.state()
    speed_limit = speed_limit
    dircol.AddConstraintToAllKnotPoints(state[2] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[2])
    dircol.AddConstraintToAllKnotPoints(state[3] <= speed_limit)
    dircol.AddConstraintToAllKnotPoints(-speed_limit <= state[3])
    (ee_y, ee_z, ee_y_dot, ee_z_dot) = ff_acromonk(state[0],state[1], state[2], state[3])
    # # Semi circle constraint about second bar
    #stick_center = [0, ladder_distance, 0]
    #dircol.AddConstraintToAllKnotPoints(ee_y <= stick_center[1])
    ## Add constraint to avoid collision to attatched bar
    theta_limit = theta_limit
    dircol.AddConstraintToAllKnotPoints(state[1] <= theta_limit)
    dircol.AddConstraintToAllKnotPoints(-theta_limit <= state[1]); 
    final_state = final_state
    
    prog.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state());
    
    dircol.AddRunningCost(R * u[0]**2)
    dircol.AddRunningCost(state[2]**2)
    dircol.AddRunningCost(state[3]**2)
    # Add a logarithmic cost for collision avoidance with stick
    # TODO: Add the barrier function > linspace out and the distance calculation in

    # stick_center = [ladder_distance, 0]
    # r = 0.4 #0.15 < r < 0.8
    # y = np.linspace(-r + stick_center[0], r+stick_center[0], 5000)
    # z = -np.sqrt(r**2 - (y - stick_center[0])**2) + stick_center[1]
    # distance = np.sqrt((y-ee_y)**2+(z-ee_z)**2)
    # x = np.linspace(distance + 0.001,2,5000)
    # y = -50 * np.log(x-distance)
    # dircol.AddRunningCost(y)
#     Add a final cost equal to the total duration.
    dircol.AddFinalCost(dircol.time()*time_panalization)    
    initial_x_trajectory = PiecewisePolynomial.FirstOrderHold(
        init_guess, np.column_stack((initial_state, final_state)))
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)
    result = Solve(prog)
    assert (result.is_success())
    hyper_params_dict={
            'n':n,
            'tau_limit':tau_limit,
            'initial_state':initial_state,
            'theta_limit':theta_limit,
            'speed_limit':speed_limit,
            'ladder_distance':ladder_distance,
            'final_state':final_state,
            'R':R,
            'time_panalization':time_panalization,
            'init_guess':init_guess,
            'max_timestep':max_timestep,
            'min_timestep':min_timestep       
    }
    return result, dircol, hyper_params_dict
    
    
class FitPiecewisePolynomial():
    '''
    Gets data and number of break points and 
    fit cubic segment polynomials to each section of data
    '''
    def __init__(self, data_x, data_y, num_break, poly_degree):
        self.data_x = data_x
        self.data_y = data_y
        self.num_break = num_break
        self.poly_degree=poly_degree
        (self.x_sec_data,
         self.y_sec_data,
         self.coeff_sec_data)=self.create_section_poly()
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
    
    def poly1(self, t,A,B):
        return A * pow(t, 1) + B 
    def poly2(self, t,A,B,C):
        return A * pow(t, 2) + B * pow(t, 1) + C 
    def poly3(self, t,A,B,C,D):
        return A * pow(t, 3) + B * pow(t, 2) + C * pow(t, 1) + D      
    
    def end_time(self):
        return self.data_x[-1]
    
    def start_time(self):
        return self.data_x[0]
    
    def split_data(self, data):
        '''
        Takes the original data and return a list of splitted arrays
        '''
        l = len(data)
        sec_len = int(np.ceil(l/self.num_break))  
#         if l % self.num_break != 0 :
#             print(f'Uneven division.len={l},section_len={sec_len}, last_sec_len={l - (sec_len * (self.num_break - 1))}')
        return np.array_split(data, self.num_break)    

    def create_section_poly(self):
        '''
        This function takes the splitted data(x, y) and return 2 lists
        - list of the x-data to be fitted to the setion data
        - list of the fitted value
        '''
        splitted_data_x = self.split_data(self.data_x)
        splitted_data_y = self.split_data(self.data_y)
        x_sec_data = []
        y_sec_data = []
        coeff_sec_data = []
        index = 0
        for sec in splitted_data_x:
            x_sec_data.append(np.linspace(sec[0],sec[-1],len(sec)))
            func = self.determin_poly()
            p_coeff, p_cov = cf(func, splitted_data_x[index],splitted_data_y[index])
            fit = func(x_sec_data[index],*p_coeff)
            y_sec_data.append(fit)
            coeff_sec_data.append(p_coeff)
            index +=1  
        self.x_sec_data=x_sec_data 
        self.y_sec_data=y_sec_data
        self.coeff_sec_data=coeff_sec_data              
        return x_sec_data, y_sec_data, coeff_sec_data   

    def get_value(self, value):
        poly_index = min([index for index, element in enumerate([any(poly >= value) for poly in self.x_sec_data])if element == True])
        p_coeff = self.coeff_sec_data[poly_index]
        func = self.determin_poly()            
        return func(value,*p_coeff)        


def create_gain_arrays(tvlqr_obj, frequency):
    n_points = int(tvlqr_obj.K.end_time() / (1 / frequency))
    time_stamp = np.linspace(tvlqr_obj.K.start_time(),
                        tvlqr_obj.K.end_time(),
                        n_points)
    K = [tvlqr_obj.K.value(i) for i in time_stamp]    
    index = 0
    K_array = np.zeros((n_points,4))
    for k_vec in K:
        K_array[index] = k_vec[0]
        index += 1
    k0 = np.array([tvlqr_obj.k0.value(i)[0][0] for i in time_stamp])
    return K_array,k0 , time_stamp


def create_nominal_pcws(data):
    breaks = 70 
    sh_des_pos_pcw = FitPiecewisePolynomial(data_x=data.des_time, 
                                            data_y=data.shoulder_des_pos, 
                                            num_break=breaks, 
                                            poly_degree=3)
    sh_des_vel_pcw = FitPiecewisePolynomial(data_x=data.des_time, 
                                            data_y=data.shoulder_des_vel, 
                                            num_break=breaks, 
                                            poly_degree=3)      
    el_des_pos_pcw = FitPiecewisePolynomial(data_x=data.des_time, 
                                            data_y=data.elbow_des_pos, 
                                            num_break=breaks, 
                                            poly_degree=3)
    el_des_vel_pcw = FitPiecewisePolynomial(data_x=data.des_time, 
                                            data_y=data.elbow_des_vel, 
                                            num_break=breaks, 
                                            poly_degree=3)  
    el_des_tau_pcw = FitPiecewisePolynomial(data_x=data.des_time, 
                                            data_y=data.elbow_des_tau, 
                                            num_break=50, 
                                            poly_degree=2)   
    el_tau_tvlqr_pcw = FitPiecewisePolynomial(data_x=data.des_time, 
                                              data_y=data.elbow_des_tau_tvlqr, 
                                              num_break=50, 
                                              poly_degree=2)

    num_break = 100
    poly_deg = 3                                              


    k0 = FitPiecewisePolynomial(data_y=data.k0, 
                                data_x=data.des_time, 
                                num_break=num_break, 
                                poly_degree=poly_deg)

    k1 = FitPiecewisePolynomial(data_y=data.k1, 
                                data_x=data.des_time, 
                                num_break=num_break, 
                                poly_degree=poly_deg)

    k2 = FitPiecewisePolynomial(data_y=data.k2, 
                                data_x=data.des_time, 
                                num_break=num_break, 
                                poly_degree=poly_deg)

    k3 = FitPiecewisePolynomial(data_y=data.k3, 
                                data_x=data.des_time, 
                                num_break=num_break, 
                                poly_degree=poly_deg)

    k4 = FitPiecewisePolynomial(data_y=data.k4, 
                                data_x=data.des_time, 
                                num_break=num_break, 
                                poly_degree=poly_deg) 
    PCSW=namedtuple('PCWS',
                    [
                    'k0',
                    'k1',
                    'k2',
                    'k3',
                    'k4',
                    'sh_des_pos_pcw',
                    'el_des_pos_pcw',
                    'sh_des_vel_pcw',
                    'el_des_vel_pcw',
                    'el_des_tau_pcw',
                    'el_tau_tvlqr_pcw'
                    ]
                    ) 
    pcws = PCSW(
                k0=k0,
                k1=k1,
                k2=k2,
                k3=k3,
                k4=k4,
                sh_des_pos_pcw=sh_des_pos_pcw,
                el_des_pos_pcw=el_des_pos_pcw,
                sh_des_vel_pcw=sh_des_vel_pcw,
                el_des_vel_pcw=el_des_vel_pcw,
                el_des_tau_pcw=el_des_tau_pcw,
                el_tau_tvlqr_pcw=el_tau_tvlqr_pcw
                )

    return pcws


def tau_tvlqr(nominal_pcws, meas_states_array, time):
    # print('tau_tvlqr:start')
    x0 = np.array([
                   [nominal_pcws.sh_des_pos_pcw.get_value(time)],
                   [nominal_pcws.el_des_pos_pcw.get_value(time)],
                   [nominal_pcws.sh_des_vel_pcw.get_value(time)],
                   [nominal_pcws.el_des_vel_pcw.get_value(time)],
                  ])
    # print(f'x0 = {x0}')
    xbar = meas_states_array - x0                  
    # print(f'xbar = {xbar}')
    K = np.array([[
                  nominal_pcws.k1.get_value(time),
                  nominal_pcws.k2.get_value(time),
                  nominal_pcws.k3.get_value(time), 
                  nominal_pcws.k4.get_value(time)
                ]])      
    # print(f'K = {K}')              
    # k0 = np.array([[nominal_pcws.k0.get_value(time)]])
    # print(f'k0 = {k0}')
    u0 = np.array([[nominal_pcws.el_des_tau_pcw.get_value(time)]])
    tau = u0 - np.dot(K,xbar) #- k0
    return tau

def tau_tvlqr_subsequent(nominal_pcws, meas_states_array, time):
    # print('tau_tvlqr:start')
    x0 = np.array([
                   [-1 * nominal_pcws.sh_des_pos_pcw.get_value(time)],
                   [-1 * nominal_pcws.el_des_pos_pcw.get_value(time)],
                   [-1 * nominal_pcws.sh_des_vel_pcw.get_value(time)],
                   [-1 * nominal_pcws.el_des_vel_pcw.get_value(time)],
                  ])
                  
    '''x0 = np.array([
                   [nominal_pcws.sh_des_pos_pcw.get_value(time)],
                   [-1 * nominal_pcws.el_des_pos_pcw.get_value(time)],
                   [nominal_pcws.sh_des_vel_pcw.get_value(time)],
                   [-1 * nominal_pcws.el_des_vel_pcw.get_value(time)],
                  ])'''
    # print(f'x0 = {x0}')
    xbar = meas_states_array - x0                  
    # print(f'xbar = {xbar}')
    K = np.array([[
                  nominal_pcws.k1.get_value(time),
                  nominal_pcws.k2.get_value(time),
                  nominal_pcws.k3.get_value(time), 
                  nominal_pcws.k4.get_value(time)
                ]])      
    # print(f'K = {K}')              
    # k0 = np.array([[nominal_pcws.k0.get_value(time)]])
    # print(f'k0 = {k0}')
    u0 = -1 * np.array([[nominal_pcws.el_des_tau_pcw.get_value(time)]])
    tau = u0 - np.dot(K,xbar) #- k0
    return tau

def plot_custom_data(data, test):
    date = datetime.now().strftime("%Y%m%d-%I%M%S-%p")
    folder_name = f'results/trajectory_replay/{test}'
    file_name = date
    directory = generate_path(folder_name, file_name, 1)
    print(f'save to {directory}')
    os.makedirs(directory)
    plot_data(x_data=[data['meas_time'], 
                      data['meas_time'], 
                      data['meas_time'], 
                      data['meas_time']],
              y_data=[data['shoulder_meas_pos'], 
                      data['shoulder_des_pos'],
                      data['elbow_meas_pos'], 
                      data['elbow_des_pos']],
              labels=["Time (s)", "Position (rad)"],
              title="Position (rad) vs Time (s)",
              legends=['sh_meas',
                       'sh_des',
                       'el_meas',
                       'el_des'],
              linestyle=['-','--','-','--'],
              colors=['blue',
                      'cornflowerblue',
                      'red',
                      'indianred'],
              save_name=directory + '/pos.pdf')

    plot_data(x_data=[data['meas_time'], 
                      data['meas_time'], 
                      data['meas_time'], 
                      data['meas_time']],
              y_data=[data['shoulder_meas_vel'], 
                      data['shoulder_des_vel'], 
                      data['elbow_meas_vel'], 
                      data['elbow_des_vel']],
              labels=["Time (s)", "Velocity (rad/s)"],
              title="Velocity (rad/s) vs Time (s)",
              legends=['sh_meas',
                       'sh_des',
                       'el_meas',
                       'el_des'],
              linestyle=['-','--','-','--'],
              colors=['blue',
                      'cornflowerblue',
                      'red',
                      'indianred'],
              save_name=directory + '/vel.pdf')

    plot_data(x_data=[data['meas_time'], 
                      data['meas_time'], 
                      data['meas_time']],
              y_data=[data['elbow_meas_tau'], 
                      data['elbow_des_tau'],
                      data['elbow_des_tau_tvlqr']],
              labels=["Time (s)", "Torque (Nm)"],
              title="Torque (Nm) vs Time (s)",
              legends=['el_meas', 
                       'el_des',
                       'el_tvlqr_sim'],
              linestyle=['-','--','--'],
              colors=['blue',
                      'cornflowerblue',
                      'aqua'],
              save_name=directory + '/tau.pdf')
    plt.show()
    return directory                


def motor_test_parameters(test,u_tvlqr,u_ff):
    if test == 'pd':
        kp_scale = 1
        kd_scale = 1
        tau_cmd = 0
    elif test == 'ff_replay':
        kp_scale = 0
        kd_scale = 0
        tau_cmd = u_ff
    elif test == 'tvlqr':
        kp_scale = 0
        kd_scale = 0
        tau_cmd = u_tvlqr            
    else:
        raise Exception("Wrong test input!")
    Data=namedtuple(
        'Data',
        [
        'kp_scale',
        'kd_scale',
        'tau_cmd'
        ]
    )
    data = Data(
                kp_scale=kp_scale,
                kd_scale=kd_scale,
                tau_cmd=tau_cmd
    )
    return data
