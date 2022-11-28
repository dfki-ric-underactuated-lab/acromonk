#!/usr/bin/env python3
import sys
import asyncio
import numpy as np
import moteus
import moteus_pi3hat
import time
import os
from utils_rpi import (
    send_rad_command,
    state_estimation,
    state_estimation_v2,
    motor_test_parameters,
    go_to_pos,
    arms_attached,
)
from utils_data import (
    data_append,
    read_data,
    save_data,
    prepare_store_data,
    prepare_des_data,
    make_results_directory,
    plot_custom_data_with_dir,
    create_nominal_pcws,
    )
from reinforcement_learning_controller import get_controller

FOLDER_NAME = "data/trajectories/closed_loop"
TAU_LIMIT = 1
TAU_CATCH = 0
TAU_KICKBACK = 0
RECOVERY_CONTROLLER = "pd"
TEST = sys.argv[1]
assert TEST in ["tvlqr", "pd", "rl", "ff_replay"]
FILE_NAME = sys.argv[2]
BUS_NUMBER = int(sys.argv[3])
MOTOR_ID = int(sys.argv[4])
FILE_NAME_RECOVERY = sys.argv[5]
IMU_CONFIG_ODD = (90, 0, 90)
CSV_DATA = read_data(folder=FOLDER_NAME, file=FILE_NAME, up_directory=3)
DATA = prepare_store_data(n=12000)
DATA_DES = prepare_des_data(csv_data=CSV_DATA)
NOMINAL_PCWS = create_nominal_pcws(DATA_DES)
RECOVERY_CSV_DATA = read_data(folder='data/trajectories/closed_loop', file=FILE_NAME_RECOVERY, up_directory=3)
DATA_RECOVERY = prepare_des_data(csv_data=RECOVERY_CSV_DATA)
FORWAR_RECOVERY_PCWS = create_nominal_pcws(DATA_RECOVERY)
INDEX = [0]


class ShutdownException(Exception):
    pass


class RecoveryException(Exception):
    pass


class BrachiationFailure(Exception):
    pass


async def generate_aa(imu, servo, brach_type):
    print('[generate_aa]:disabled for now')
    return 1, 'v1'
    #await servo.set_stop()
    number_of_loops = 10
    aa, conf =  await (arms_attached(imu, servo,number_of_loops, brach_type))
    print(f'[generate_aa]:aa={aa}, conf={conf}')
    return aa, conf


def run(f):
    return loop.run_until_complete(f)


def kb_cond(brach, t, el_vel):
    return (
        (t < 0.05 or -1.3 < el_vel)
        if brach == "even"
        else (t < 0.05 or el_vel < 1.3)
    )


async def kb_state_estimate(brach, pr, el_pos, el_vel):
    return (
        (
            await state_estimation(
                pr=pr, pos_el=el_pos, vel_el=el_vel
            )
        )
        if brach == "odd"
        else (await state_estimation_v2(pr))
    )


async def Kickback(
    n, torque, tau_limit_KB, servo, pr, brach, t0_experiment
):  
    success = False
    el_vel = 94305

    index_KB = 0
    meas_dt = 0
    t = 0

    await servo.set_stop()
    while kb_cond(brach, t, el_vel):
        start_loop = time.time()
        t += meas_dt
        tau_cmd = np.clip(torque, -tau_limit_KB, tau_limit_KB)

        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,
            vel=0.0,
            tau=tau_cmd,
            tau_limit=tau_limit_KB,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=float("nan"),
        )

        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await kb_state_estimate(
            brach, pr, el_pos, el_vel
        )
        # store measured data
        data_append(
            DATA,
            phase="kickback",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            SH_MEAS_POS=sh_pos,
            SH_MEAS_VEL=sh_vel,
            EL_MEAS_POS=el_pos,
            EL_MEAS_VEL=el_vel,
            EL_CMD_TAU=torque,
            EL_MEAS_TAU=el_tau,
            EL_CLIPPED_TAU=tau_cmd,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            EL_DES_POS=np.nan,
            EL_DES_VEL=np.nan,
            SH_DES_POS=np.nan,
            SH_DES_VEL=np.nan,
        )
        index_KB += 1
        INDEX[0] += 1
        meas_dt = time.time() - start_loop
    print(
        f"kickback loop control frequency = {1/meas_dt}, index_KB={index_KB}"
    )
    if abs(el_vel) != 94305:
        success = True
    # Disabling the motor
    await servo.set_stop()
    return success


def forward_recovery(brach, servo, imu, t0_experiment, controller):
    # print('[forward recovery]: disabled for now')
    # return 
    run(servo.set_stop())
    run(go_to_pos(servo, 0.0, rad_per_cycle=0.001, tau_limit = 1))
    meas_dt = 0
    sign = 1 if brach == "odd" else -1
    t = 0
    sh_pos = FORWAR_RECOVERY_PCWS.sh_des_pos_pcw.get_value(t)
    sh_vel = FORWAR_RECOVERY_PCWS.sh_des_vel_pcw.get_value(t)
    el_pos = FORWAR_RECOVERY_PCWS.el_des_pos_pcw.get_value(t)
    el_vel = FORWAR_RECOVERY_PCWS.el_des_vel_pcw.get_value(t)
    meas_dt = 0
    while t < DATA_RECOVERY.des_time[-1]:
        t_iteration = time.time()
        t += meas_dt
        index = INDEX[0]
        meas_state = np.array([[sh_pos], [el_pos], [sh_vel], [el_vel]])
        if t > DATA_RECOVERY.des_time[-1]:
            break
        el_des_pos = sign * FORWAR_RECOVERY_PCWS.el_des_pos_pcw.get_value(t)
        el_des_vel = sign * FORWAR_RECOVERY_PCWS.el_des_vel_pcw.get_value(t)
        sh_des_pos = FORWAR_RECOVERY_PCWS.sh_des_pos_pcw.get_value(t)
        sh_des_vel = FORWAR_RECOVERY_PCWS.sh_des_vel_pcw.get_value(t)
        des_state = np.array(
            [[sh_des_pos], [el_des_pos], [sh_des_vel], [el_des_vel]]
        )

        motor_params = motor_test_parameters(
            test=RECOVERY_CONTROLLER,
            nominal_pcws=FORWAR_RECOVERY_PCWS,
            brach_sign=sign,
            meas_state=meas_state,
            des_state=des_state,
            time=t,
            controller=controller,
        )
        clipped_tau = sign * np.clip(
            motor_params.tau_cmd, -TAU_LIMIT, TAU_LIMIT
        )
        if RECOVERY_CONTROLLER == "tvlqr":
            clipped_tau = np.clip(motor_params.tau_cmd, -TAU_LIMIT, TAU_LIMIT)
        (el_pos, el_vel, el_tau) = run(
            send_rad_command(
                controller_obj=servo,
                pos=el_des_pos,
                vel=el_des_vel,
                tau=clipped_tau,
                tau_limit=TAU_LIMIT,
                kp_scale=motor_params.kp_scale,
                kd_scale=motor_params.kd_scale,
                watchdog_timeout=0.05,
            )
        )
        if brach == "odd":
            state = run(
                state_estimation(
                    pr=imu,
                    pos_el=el_pos,
                    vel_el=el_vel
                )
            )
        else:
            state = run(state_estimation_v2(pr=imu))
        (sh_pos, sh_vel, raw_imu_pos, raw_imu_vel) = state
        K = motor_params.K_tvlqr
        # store measured data
        data_append(
            DATA,
            phase="recovery",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            SH_MEAS_POS=sh_pos,
            SH_MEAS_VEL=sh_vel,
            EL_MEAS_POS=el_pos,
            EL_MEAS_VEL=el_vel,
            EL_CMD_TAU=motor_params.tau_cmd,
            EL_MEAS_TAU=el_tau,
            EL_CLIPPED_TAU=clipped_tau,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            EL_DES_POS=el_des_pos,
            EL_DES_VEL=el_des_vel,
            SH_DES_POS=sh_des_pos,
            SH_DES_VEL=sh_des_vel,
            TVLQR_K1=K[0][0],
            TVLQR_K2=K[0][1],
            TVLQR_K3=K[0][2],
            TVLQR_K4=K[0][3],
        )
        INDEX[0] += 1
        meas_dt = time.time() - t_iteration
    run(servo.set_stop())        
    print(f"recovery loop control frequency = {1/meas_dt}")


def check_arms_attached(imu, servo, number_of_loops, brach_type):
    aa = run(arms_attached(imu, servo, number_of_loops, brach_type))
    print(f'[check_arms_attached]:aa={aa}')
    if aa == 0:
        raise ShutdownException()
    elif aa == 1:
        raise RecoveryException()


def kickback(
    brach, n, tau, tau_limit, servo, imu, t0_experiment
):

    tau = tau if brach == "odd" else -tau
    print('[kickback: disabled for now]')
    return    
    success = run(
        Kickback(
            n, tau, tau_limit, servo, imu, brach, t0_experiment
        )
    )
    if not success:
        raise BrachiationFailure(
            "kickback",
            brach,
            n,
            tau,
            tau_limit,
            imu,
            t0_experiment,
        )


async def ch_state_estimate(brach, pr, el_pos, el_vel):
    return (
        await state_estimation(
            pr=pr, pos_el=el_pos, vel_el=el_vel
        )
        if brach == "odd"
        else (await state_estimation_v2(pr))
    )


async def Catch(
    idling_time,
    catch_duration,
    torque_CATCH,
    tau_limit_CATCH,
    servo,
    pr,
    brach,
    t0_experiment,
):
    # success = False
    success = True
    await servo.set_stop()
    index_CATCH = 0
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
            pos=0.0,
            vel=0.0,
            tau=idling_tau,
            tau_limit=tau_limit_CATCH,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )

        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await ch_state_estimate(
            brach, pr, el_pos, el_vel
        )
        # store measured data
        data_append(
            DATA,
            phase="catch",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            SH_MEAS_POS=sh_pos,
            SH_MEAS_VEL=sh_vel,
            EL_MEAS_POS=el_pos,
            EL_MEAS_VEL=el_vel,
            EL_CMD_TAU=torque_CATCH,
            EL_MEAS_TAU=el_tau,
            EL_CLIPPED_TAU=torque_CATCH,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            EL_DES_POS=np.nan,
            EL_DES_VEL=np.nan,
            SH_DES_POS=np.nan,
            SH_DES_VEL=np.nan,
        )
        INDEX[0] += 1
        meas_dt_CATCH = time.time() - start_loop
    catch_start = time.time()
    while time.time() - catch_start < catch_duration:
        start_loop = time.time()
        t_CATCH += meas_dt_CATCH

        tau_cmd_CATCH = np.clip(
            torque_CATCH, -tau_limit_CATCH, tau_limit_CATCH
        )
        # send zero command to motor to make sure nothing would happen
        (el_pos, el_vel, el_tau) = await send_rad_command(
            controller_obj=servo,
            pos=0.0,
            vel=0.0,
            tau=tau_cmd_CATCH,
            tau_limit=tau_limit_CATCH,
            kp_scale=0.0,
            kd_scale=0.0,
            watchdog_timeout=0.05,
        )
        sh_pos, sh_vel, raw_imu_pos, raw_imu_vel = await ch_state_estimate(
            brach, pr, el_pos, el_vel
        )
        # store measured data
        data_append(
            DATA,
            phase="catch",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            SH_MEAS_POS=sh_pos,
            SH_MEAS_VEL=sh_vel,
            EL_MEAS_POS=el_pos,
            EL_MEAS_VEL=el_vel,
            EL_CMD_TAU=torque_CATCH,
            EL_MEAS_TAU=el_tau,
            EL_CLIPPED_TAU=tau_cmd_CATCH,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            EL_DES_POS=np.nan,
            EL_DES_VEL=np.nan,
            SH_DES_POS=np.nan,
            SH_DES_VEL=np.nan,
        )
        index_CATCH += 1
        INDEX[0] += 1
        meas_dt_CATCH = time.time() - start_loop
    print(f"Catch loop control frequency = {1/meas_dt_CATCH}")
    success = True
    await servo.set_stop()
    return success


def catch(
    brach,
    idling_time,
    catch_duration,
    tau,
    tau_limit,
    servo,
    imu,
    t0_experiment,
):
    tau = tau if brach == "odd" else -tau
    print('[catch: disabled for now]')
    return
    success = run(
        Catch(
            idling_time,
            catch_duration,
            tau,
            tau_limit,
            servo,
            imu,
            brach,
            t0_experiment,
        )
    )
    if not success:
        raise BrachiationFailure(
            "catch",
            brach,
            idling_time,
            catch_duration,
            tau,
            tau_limit,
            imu,
            t0_experiment,
        )


def swing(brach, servo, imu, t0_experiment, controller):
    print('[swing: disabled for now]')
    return
    run(servo.set_stop())
    meas_dt = 0
    sign = 1 if brach == "odd" else -1
    t = 0
    sh_pos = NOMINAL_PCWS.sh_des_pos_pcw.get_value(t)
    sh_vel = NOMINAL_PCWS.sh_des_vel_pcw.get_value(t)
    el_pos = NOMINAL_PCWS.el_des_pos_pcw.get_value(t)
    el_vel = NOMINAL_PCWS.el_des_vel_pcw.get_value(t)
    meas_dt = 0
    while t < DATA_DES.des_time[-1]:
        t_iteration = time.time()
        t += meas_dt
        index = INDEX[0]
        meas_state = np.array([[sh_pos], [el_pos], [sh_vel], [el_vel]])
        if t > DATA_DES.des_time[-1]:
            break
        el_des_pos = sign * NOMINAL_PCWS.el_des_pos_pcw.get_value(t)
        el_des_vel = sign * NOMINAL_PCWS.el_des_vel_pcw.get_value(t)
        sh_des_pos = NOMINAL_PCWS.sh_des_pos_pcw.get_value(t)
        sh_des_vel = NOMINAL_PCWS.sh_des_vel_pcw.get_value(t)
        des_state = np.array(
            [[sh_des_pos], [el_des_pos], [sh_des_vel], [el_des_vel]]
        )

        motor_params = motor_test_parameters(
            test=TEST,
            nominal_pcws=NOMINAL_PCWS,
            brach_sign=sign,
            meas_state=meas_state,
            des_state=des_state,
            time=t,
            controller=controller,
        )
        clipped_tau = sign * np.clip(
            motor_params.tau_cmd, -TAU_LIMIT, TAU_LIMIT
        )
        if TEST == "tvlqr":
            clipped_tau = np.clip(motor_params.tau_cmd, -TAU_LIMIT, TAU_LIMIT)
        (el_pos, el_vel, el_tau) = run(
            send_rad_command(
                controller_obj=servo,
                pos=el_des_pos,
                vel=el_des_vel,
                tau=clipped_tau,
                tau_limit=TAU_LIMIT,
                kp_scale=motor_params.kp_scale,
                kd_scale=motor_params.kd_scale,
                watchdog_timeout=0.05,
            )
        )
        if brach == "odd":
            state = run(
                state_estimation(
                    pr=imu,
                    pos_el=el_pos,
                    vel_el=el_vel
                )
            )
        else:
            state = run(state_estimation_v2(pr=imu))
        (sh_pos, sh_vel, raw_imu_pos, raw_imu_vel) = state
        K = motor_params.K_tvlqr
        # store measured data
        data_append(
            DATA,
            phase="swing",
            index=INDEX[0],
            TIME=time.time() - t0_experiment,
            SH_MEAS_POS=sh_pos,
            SH_MEAS_VEL=sh_vel,
            EL_MEAS_POS=el_pos,
            EL_MEAS_VEL=el_vel,
            EL_CMD_TAU=motor_params.tau_cmd,
            EL_MEAS_TAU=el_tau,
            EL_CLIPPED_TAU=clipped_tau,
            RAW_IMU_POS=raw_imu_pos,
            RAW_IMU_VEL=raw_imu_vel,
            EL_DES_POS=el_des_pos,
            EL_DES_VEL=el_des_vel,
            SH_DES_POS=sh_des_pos,
            SH_DES_VEL=sh_des_vel,
            TVLQR_K1=K[0][0],
            TVLQR_K2=K[0][1],
            TVLQR_K3=K[0][2],
            TVLQR_K4=K[0][3],
        )
        INDEX[0] += 1
        meas_dt = time.time() - t_iteration
    run(servo.set_stop())    
    print(f"swing loop control frequency = {1/meas_dt}")


def check_aa(aa):
    if aa == 0:
        raise ShutdownException()
    elif aa == 1:
        raise RecoveryException()


def version_conflict(before, after):
    return True if before != after else False


async def wait(next_move, aa, current_conf=None, brach_type=None, seconds=0):
    return
    sign = 1 if brach_type == 'odd' else -1
    R_final_pos = FORWAR_RECOVERY_PCWS.el_des_pos_pcw.get_value(FORWAR_RECOVERY_PCWS.el_des_pos_pcw.end_time())
    swing_final_pos = R_final_pos = NOMINAL_PCWS.el_des_pos_pcw.get_value(NOMINAL_PCWS.el_des_pos_pcw.end_time())
    final_pos = R_final_pos if next_move == 'recovery' else swing_final_pos
    print(f'next move in {seconds} seconds:{next_move}, aa={aa}')#, conf=v{current_conf},motor_final_pos={sign * final_pos}')
    await (asyncio.sleep(seconds))


def robust_forward_brachiation(
    loop,
    ibrachiation,
    brach,
    t0,
    servo,
    imu,
    controller,    
    n_max_recovery_try=3,
    ):
    aa_init, conf = run(generate_aa(imu, servo, brach_type=brach))
    recovery_flag = False
    BF_flag = False
    try:
        print(f'1st try block:check_aa={aa_init}')
        check_aa(aa_init)
    except RecoveryException:
        print('Recovery Exception')
        for i in range(n_max_recovery_try):
            try:
                aa_R, before_R_conf = run(generate_aa(imu, servo, brach_type=brach))
                forward_recovery(brach, servo, imu, t0, controller=None)
                print('forward recovery before swing')
                recovery_flag = True
                aa_R, after_R_conf = run(generate_aa(imu, servo, brach_type=brach))
                R_version_conflict = version_conflict(before_R_conf, after_R_conf)
                print('check after recovery= ', check_aa(aa_R))
                if recovery_flag:
                    print(
                        f"[catch after recovery] ibrach:{ibrachiation} brach:{brach} time.time()-t0:{time.time()-t0}"
                    )            
                    catch(
                        brach=brach,
                        idling_time=0.2,
                        catch_duration=0.1,
                        tau=TAU_CATCH,
                        tau_limit=1.2,
                        servo=servo,
                        imu=imu,
                        t0_experiment=t0,
                    )
                    print(f'\nSuccessful recovery on {i+1} try.')
                    return
            except RecoveryException:
                if R_version_conflict:
                    print(f'\nAutomatic Detach After {i+1} Recovery.(before_R_conf, after_R_conf)={before_R_conf, after_R_conf}')
                    return
                else:
                    print(f'\nRecovery failed in {i+1} try.')
                    continue
        print(f'\nMaximum trials of {n_max_recovery_try} for recovery reached.')        
        raise ShutdownException()            
    _, before_BF_conf = run(generate_aa(imu, servo, brach_type=brach))
    print(
        f"[kickback] ibrach:{ibrachiation} brach:{brach} time.time()-t0:{time.time()-t0}"
    )
    kickback(brach, 49, TAU_KICKBACK, 3.0, servo, imu, t0)    
    print(
        f"[swing] ibrach:{ibrachiation} brach:{brach} time.time()-t0:{time.time()-t0}"
    )            
    swing(brach, servo, imu, t0, controller)    
    BF_flag = True
    aa_after_swing, after_BF_conf = run(generate_aa(imu, servo, brach_type=brach))
    BF_version_conflict = version_conflict(before_BF_conf, after_BF_conf)
    try:
        check_aa(aa_after_swing)
        if BF_flag and BF_version_conflict:
            print(
                f"[catch after swing] ibrach:{ibrachiation} brach:{brach} time.time()-t0:{time.time()-t0}"
            )           
            catch(
                brach=brach,
                idling_time=0.2,
                catch_duration=0.1,
                tau=TAU_CATCH,
                tau_limit=1.2,
                servo=servo,
                imu=imu,
                t0_experiment=t0,
            )
            print('\nSuccessful BF.')
        else:
            print('\nWierd Exception!')
            raise ShutdownException()
    except RecoveryException:
        if BF_flag and BF_version_conflict:
            print('\nAutomatic Detach After Swing.')
            return
        for i in range(n_max_recovery_try):
            try:
                aa_R, before_R_conf = run(generate_aa(imu, servo, brach_type=brach))
                forward_recovery(brach, servo, imu, t0, controller=None)
                print('forward recovery after swing')                
                recovery_flag = True
                aa_R, after_R_conf = run(generate_aa(imu, servo, brach_type=brach))
                R_version_conflict = version_conflict(before_R_conf, after_R_conf)
                check_aa(aa_R)
                if recovery_flag:
                    print(
                        f"[catch after recovery] ibrach:{ibrachiation} brach:{brach} time.time()-t0:{time.time()-t0}"
                    )            
                    catch(
                        brach=brach,
                        idling_time=0.2,
                        catch_duration=0.1,
                        tau=TAU_CATCH,
                        tau_limit=1.2,
                        servo=servo,
                        imu=imu,
                        t0_experiment=t0,
                    )
                    print(f'\nSuccessful recovery on {i+1} try.')
                    return
            except RecoveryException:
                if R_version_conflict:
                    print(f'\nAutomatic Detach After {i+1} Recovery.')
                    return
                else:
                    print(f'\nRecovery failed in {i+1} try.')
                    continue
        print(f'\nMaximum trials of {n_max_recovery_try} for recovery reached.')        
        raise ShutdownException()      
    time.sleep(0.2)


def imu_reset(con, sleep_time=3):
    moteus_pi3hat.Pi3HatRouter(mounting_deg={"roll": 20, "pitch": 0, "yaw": 0})
    imu = moteus_pi3hat.Pi3HatRouter(
        mounting_deg={"roll": con[0], "pitch": con[1], "yaw": con[2]}
    )
    time.sleep(sleep_time)
    return imu


async def init():
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            BUS_NUMBER: [MOTOR_ID],
        },
    )
    servo = moteus.Controller(id=MOTOR_ID, transport=transport)
    imu = imu_reset(IMU_CONFIG_ODD)
    await servo.set_stop()
    if TEST == "rl":
        print("Loading RL controller")
        controller = get_controller()
    else:
        controller = None
    return servo, imu, controller


def main(loop):
    nbrachiation = int(os.getenv("NBRACH", "1"))
    INIT_CONF = int(sys.argv[6])
    try:
        for ibrachiation in range(
            INIT_CONF, nbrachiation + 1
        ):
            print(f"[main] ibrachiation:{ibrachiation}")

            brach = "even" if ibrachiation % 2 == 0 else "odd"

            if (
                ibrachiation == INIT_CONF
            ):
                (
                    servo,
                    imu,
                    controller,
                ) = loop.run_until_complete(init())
                run(go_to_pos(servo, 0.0, rad_per_cycle=0.001, tau_limit = 1))
                if input("Start? (y/n) ").lower() != "y":
                    exit(1)
                t0 = time.time()
            robust_forward_brachiation(
                loop,
                ibrachiation,
                brach,
                t0,
                servo,
                imu,
                controller,
                n_max_recovery_try=3,
                )                            
    except ShutdownException:
        print("[main] Safe Shutdown!\n")
    finally:
        index = INDEX[0]
        valid_data = {
            field: DATA[i][:index]
            for i, field in enumerate(DATA._fields)
            if isinstance(DATA[i], np.ndarray)
        }
        directory = make_results_directory(f'BF_{TEST}_{nbrachiation}x')
        # Save Trajectory to a csv file to be sent to the motor.
        save_data(valid_data, directory + f"/BF_{TEST}.csv")
        # Plot the simulation results
        plot_custom_data_with_dir(directory, valid_data, show=False)
        # Disabling the motor
        run(servo.set_stop())
        print("Motor disabled.")


loop = asyncio.get_event_loop()
main(loop)
