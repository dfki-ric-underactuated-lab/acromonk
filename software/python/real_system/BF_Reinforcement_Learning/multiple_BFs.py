#!/usr/bin/env python3

import asyncio
import numpy as np
import moteus
import moteus_pi3hat
import time
import os
import sys
from scipy.spatial import ConvexHull
from utils import send_rad_command, read_imu_data, zero_offset, point_in_hull, bcolors, get_empirical_roa_points

from rl_controller import RLearningController
from state_estimator import StateEstimator

FOLDER_NAME = "data/trajectories"
RECORD_DATA = True
SHOW_PLOTS = False
TAU_LIMIT = 3.0

TEST = sys.argv[1]
assert TEST in ["rl"]
BUS_NUMBER = int(sys.argv[2])
MOTOR_ID = int(sys.argv[3])
TARGET_CONTROL_FREQUENCY = 80  # number [Hz] or None for as fast as possible

IMU_CONFIG_ODD = (90, 0, 90)

roa_points_absolute = get_empirical_roa_points()
ROA_HULL = ConvexHull(roa_points_absolute)


def get_brach_sign(brach):
    if brach == "odd":
        brach_sign = 1
    elif brach == "even":
        brach_sign = -1
    return brach_sign


class BrachiationFailure(Exception):
    pass


def run(f):
    return loop.run_until_complete(f)


def kb_cond(brach, t, el_vel):
    return (
        (t < 0.05 or -1.3 < el_vel)
        if brach == "even"
        else (t < 0.05 or el_vel < 1.3)
    )


async def Kickback(n, torque, tau_limit_KB, servo, brach, t0_experiment, state_estimator):
    brach_sign = get_brach_sign(brach)

    success = False
    el_vel = 94305

    meas_dt = 0
    t = 0

    while kb_cond(brach, t, el_vel):
        start_loop = time.time()
        t += meas_dt

        tau_cmd = np.clip(torque, -tau_limit_KB, tau_limit_KB)

        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,  # although 0, kp = 0 gives pos no affect
            vel=0.0,  # although 0, kd = 0 gives vel no affect
            tau=tau_cmd,
            tau_limit=tau_limit_KB,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=float("nan"),
        )

        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await state_estimator.get_shoulder_state(brach_sign=brach_sign,
                                                                                            th_2=el_pos,
                                                                                            th_2_vel=el_vel)

        meas_dt = time.time() - start_loop

    if abs(el_vel) != 94305:
        success = True

    # Disabling the motor
    await servo.set_stop()

    return success


def kickback(brach, n, tau, tau_limit, servo, t0_experiment, state_estimator):
    brach_sign = get_brach_sign(brach)
    tau *= brach_sign
    success = run(Kickback(n, tau, tau_limit, servo, brach, t0_experiment, state_estimator))
    if not success:
        raise BrachiationFailure("kickback", brach, n, tau, tau_limit, t0_experiment)


async def Catch(idling_time, catch_duration, torque_CATCH, tau_limit_CATCH, servo, brach, t0_experiment,
                state_estimator):
    # success = False
    brach_sign = get_brach_sign(brach)
    success = True  # FIXME the catch condition has problem and we disabled it for now
    t_CATCH = 0
    meas_dt_CATCH = 0
    idling_tau = 0
    idling_start = time.time()
    while time.time() - idling_start < idling_time:
        start_loop = time.time()
        t_CATCH += meas_dt_CATCH
        # send zero command to motor to make sure nothing would happen
        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,  # although 0, kp = 0 gives pos no affect
            vel=0.0,  # although 0, kd = 0 gives vel no affect
            tau=idling_tau,
            tau_limit=tau_limit_CATCH,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )

        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await state_estimator.get_shoulder_state(brach_sign=brach_sign,
                                                                                            th_2=el_pos,
                                                                                            th_2_vel=el_vel)

        meas_dt_CATCH = time.time() - start_loop

    catch_start = time.time()
    while time.time() - catch_start < catch_duration:
        start_loop = time.time()
        t_CATCH += meas_dt_CATCH

        tau_cmd_CATCH = np.clip(torque_CATCH, -tau_limit_CATCH, tau_limit_CATCH)

        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,  # although 0, kp = 0 gives pos no affect
            vel=0.0,  # although 0, kd = 0 gives vel no affect
            tau=tau_cmd_CATCH,
            tau_limit=tau_limit_CATCH,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )

        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await state_estimator.get_shoulder_state(brach_sign=brach_sign,
                                                                                            th_2=el_pos,
                                                                                            th_2_vel=el_vel)

        meas_dt_CATCH = time.time() - start_loop

    success = True
    await servo.set_stop()

    return success


def catch(brach, idling_time, catch_duration, tau, tau_limit, servo, t0_experiment, state_estimator):
    brach_sign = get_brach_sign(brach)
    tau *= brach_sign
    success = run(Catch(idling_time, catch_duration, tau, tau_limit, servo, brach, t0_experiment, state_estimator))

    if not success:
        raise BrachiationFailure("catch", brach, idling_time, catch_duration, tau, tau_limit, t0_experiment)


def swing_success(controller_type, t, sh_pos, el_pos):
    goal_reached = point_in_hull(np.array([sh_pos, el_pos]), ROA_HULL, tolerance=1e-12)

    if goal_reached:
        print(f'{bcolors.OKGREEN}Swing Goal reached, t:{t}, sh_pos: {sh_pos}, el_pos: {el_pos}{bcolors.ENDC}')

    return goal_reached


async def Swing(brach, servo, t0_experiment, controller, state_estimator):
    brach_sign = get_brach_sign(brach)
    t = 0
    t_max_iteration = 2.3

    # get the initial state estimation.
    el_pos, el_vel, _ = await send_rad_command(controller_obj=servo, pos=0.0,
                                               vel=0.0, tau=0.0,
                                               tau_limit=TAU_LIMIT, kp_scale=0.0, kd_scale=0.0)
    sh_pos, sh_vel, _, _ = await state_estimator.get_shoulder_state(brach_sign=brach_sign,
                                                                    th_2=el_pos,
                                                                    th_2_vel=el_vel)

    meas_dt = 0

    # control loop
    while not swing_success(controller.controller_type, t, sh_pos, brach_sign * el_pos) and t < t_max_iteration:
        t_iteration = time.time()
        t += meas_dt

        meas_state = np.array([[sh_pos], [el_pos], [sh_vel], [el_vel]])

        # get controller commands
        tau_cmd, des_traj_vals, kp_scale, kd_scale, spec_info = controller.get_control_command(meas_state,
                                                                                               t,
                                                                                               brach_sign)

        # get K values if applicable
        if controller.controller_type == 'tvlqr':
            k1, k2, k3, k4 = spec_info['K']

        # enforce global torque limit
        clipped_tau = np.clip(tau_cmd, -TAU_LIMIT, TAU_LIMIT)

        # write command to motor and get elbow state readings
        (el_pos, el_vel, el_tau) = await send_rad_command(controller_obj=servo, pos=des_traj_vals['el_des_pos'],
                                                          vel=des_traj_vals['el_des_vel'], tau=clipped_tau,
                                                          tau_limit=TAU_LIMIT, kp_scale=kp_scale, kd_scale=kd_scale)

        # get state estimation
        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await state_estimator.get_shoulder_state(brach_sign=brach_sign,
                                                                                            th_2=el_pos,
                                                                                            th_2_vel=el_vel)


        # idling to get to desired control frequency.
        # ATTENTION: This method with while is preferable to time.sleep(), seems more accurate!
        if TARGET_CONTROL_FREQUENCY is not None:
            while time.time() - t_iteration < 1 / TARGET_CONTROL_FREQUENCY:
                pass
        meas_dt = time.time() - t_iteration
        print(f'swing loop control frequency = {1 / meas_dt}')
    print(f'final sh: {sh_pos}, final sh_vel: {sh_vel}, final el: {el_pos}, final el_vel: {el_vel},  final time {t}')
    await servo.set_stop()


def swing(brach, servo, t0_experiment, controller, state_estimator):
    run(Swing(brach, servo, t0_experiment, controller, state_estimator))


def imu_reset(con, sleep_time=3):
    moteus_pi3hat.Pi3HatRouter(mounting_deg={"roll": 20, "pitch": 0, "yaw": 0})
    imu = moteus_pi3hat.Pi3HatRouter(
        mounting_deg={"roll": con[0], "pitch": con[1], "yaw": con[2]}
    )
    time.sleep(sleep_time)
    return imu


async def init():
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map={BUS_NUMBER: [MOTOR_ID], }, )
    servo = moteus.Controller(id=MOTOR_ID, transport=transport)
    imu = imu_reset(IMU_CONFIG_ODD)  # imu_reset(IMU_CONFIG_EVEN)
    _, _, _, init_euler_xyz = await read_imu_data(imu)
    await servo.set_stop()

    print('Loading swing phase controller...')
    if TEST == 'rl':
        print('Loading RL controller!')
        controller = RLearningController()
    else:
        raise ValueError(f'Controller type {TEST} not known, available here are "rl".')

    print('Getting state estimator ...')
    state_estimator = StateEstimator(pr=imu)

    print('Init done, ready to go!')

    return servo, controller, state_estimator


def brachiation(loop, ibrach, brach, t0, servo, controller, state_estimator):
    print(f"[brachiation] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time() - t0}")
    print(f"[kickback] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time() - t0}")
    kickback(brach, 49, 2.5, 3.0, servo, t0, state_estimator)
    print(f"[swing] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time() - t0}")
    swing(brach, servo, t0, controller, state_estimator)
    print(f"[catch] ibrach:{ibrach} brach:{brach} time.time()-t0:{time.time() - t0}")
    catch(brach, 0.4, 0.4, 0.8, 1.2, servo, t0, state_estimator)

    time.sleep(0.2)


def main(loop):
    nbrachiation = int(os.getenv("NBRACH", "3"))
    try:
        for ibrachiation in range(2, nbrachiation + 1):  # To test one brach with IMU arm attach, set range 2
            print(f"[main] ibrachiation:{ibrachiation}")

            brach = "even" if ibrachiation % 1 == 0 else "odd"

            if ibrachiation == 1:  # To test one brach with IMU arm attach, set ibrachiation == 2
                servo, controller, state_estimator = loop.run_until_complete(init())
                if input("Start? (y/n) ").lower() != "y":
                    exit(1)
                t0 = time.time()
            brachiation(loop, ibrachiation, brach, t0, servo, controller, state_estimator)
    finally:
        # Disabling the motor
        run(servo.set_stop())
        print("Motor disabled.")


zero_offset(BUS_NUMBER, MOTOR_ID)
loop = asyncio.get_event_loop()
main(loop)
