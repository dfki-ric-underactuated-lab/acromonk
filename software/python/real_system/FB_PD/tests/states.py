from mimetypes import init


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

    import asyncio
    import moteus
    import moteus_pi3hat
    import time

    # Local imports
    from utils import (send_rad_command,
                       read_imu_data,
                       state_estimation,
                       save_data,
                       read_data,
                       prepare_data,
                       zero_offset,
                       create_nominal_pcws,
                       tau_tvlqr,
                       tau_tvlqr_subsequent,
                       motor_test_parameters,
                       generate_path,
                       plot_data,
                       state_estimation_v2,
                       prepare_empty_arrays
                       )


except BaseException as e:
    print(e)

'''class IterData(type):
    def __iter__(cls):
        return iter(cls._registry)'''

class RecordData:

    #__metaclass__ = IterData
    #_registry = []
    
    def __init__(self, n=0, csv_data=None):
    
        #self._registry.append(self)
    
        if n != 0:
            self.des_time = np.zeros(n)
            self.shoulder_des_pos = np.zeros(n)
            self.shoulder_des_vel = np.zeros(n)
            self.shoulder_des_acc = np.zeros(n)
            self.shoulder_des_jerk = np.zeros(n)
            self.shoulder_des_tau = np.zeros(n)
            ## elbow.values
            self.elbow_des_pos = np.zeros(n)
            self.elbow_des_vel = np.zeros(n)
            self.elbow_des_acc = np.zeros(n)
            self.elbow_des_jerk = np.zeros(n)
            self.elbow_des_tau = np.zeros(n)
            self.elbow_des_tau_tvlqr = np.zeros(n)
            # K values
            self.k1 = np.zeros(n)
            self.k2 = np.zeros(n)
            self.k3 = np.zeros(n)
            self.k4 = np.zeros(n)
            self.k0 = np.zeros(n)
            self.dt = 0

        else:
            n = len(csv_data) 
            self.des_time = csv_data["time"].values
            self.shoulder_des_pos = csv_data["shoulder_pos"].values
            self.shoulder_des_vel = csv_data["shoulder_vel"].values
            self.shoulder_des_acc = csv_data["shoulder_acc"].values
            self.shoulder_des_jerk = csv_data["shoulder_jerk"].values
            self.shoulder_des_tau = csv_data["shoulder_torque"].values
            ## elbow.values
            self.elbow_des_pos = csv_data["elbow_pos"].values
            self.elbow_des_vel = csv_data["elbow_vel"].values
            self.elbow_des_acc = csv_data["elbow_acc"].values
            self.elbow_des_jerk = csv_data["elbow_jerk"].values
            self.elbow_des_tau = csv_data["elbow_torque"].values
            self.elbow_des_tau_tvlqr = csv_data["elbow_torque_tvlqr"].values
            # K values
            self.k1 = csv_data["K1"].values
            self.k2 = csv_data["K2"].values
            self.k3 = csv_data["K3"].values
            self.k4 = csv_data["K4"].values
            self.k0 = csv_data["k0"].values
            self.dt = csv_data["time"][2] - csv_data["time"][1]
            
        self.shoulder_meas_pos = np.zeros(n)
        self.shoulder_meas_vel = np.zeros(n)
        self.elbow_meas_pos = np.zeros(n)
        self.elbow_meas_vel = np.zeros(n)
        self.elbow_meas_tau =  np.zeros(n)
        self.elbow_cmd_tau =  np.zeros(n)        
        self.meas_time = np.zeros(n)

    '''def __iter__(self):
        return self'''
    
    def truncate(self, index):  #do not need to add one to index because it is added at end of control loop before the next loop
        #print(dir(self))
        for dataType in dir(self):
            if not dataType.startswith('__') and not callable(getattr(self, dataType)) and not dataType=='dt':
                    #print("type :", dataType)
                    #print("content :", str(getattr(self, dataType)))
                    #print("size :", len(getattr(self, dataType)))
                    temp = getattr(self, dataType)
                    setattr(self, dataType,temp[:index])
                    #print("new content :", str(getattr(self, dataType)))
                    #print("new size :", len(getattr(self, dataType)))
        #print(dir(self))
        return self

async def SupportArmUpIMUStateInit(servo):                                                  
    con = (90,0,90) # usb down
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll' : con[0],
                                                  'pitch': con[1],
                                                  'yaw'  : con[2]})
    time.sleep(2.5)                                               
    _, _, _, init_euler_xyz = await read_imu_data(pr)                       
    await servo.set_stop()

    return init_euler_xyz, pr
    
async def LeadArmUpIMUStateInit(servo):
    con = (-90,0,-90) # usb up
    # create a moteus_pi3hat.Pi3HatRouter for IMU reading
    pr = moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll': con[0],
                                                'pitch': con[1],
                                                'yaw': con[2]})
    time.sleep(2.5)
    
    return pr

async def ZeroToLeftBarMotion(attempt, csv_data, test, tau_limit, servo, init_euler_xyz, pr):

    success = False
    finalTime = 0

    # prepare data for moteus controller
    data = prepare_data(csv_data)    
    nominal_pcws= create_nominal_pcws(data)
    recordedData = RecordData(csv_data=csv_data)

    # runnig loop variables    
    index = 0
    t = 0
    meas_dt = 0
    time_exceed_counter = 0

    # inital state array
    sh_pos = nominal_pcws.sh_des_pos_pcw.get_value(t)
    sh_vel = nominal_pcws.el_des_pos_pcw.get_value(t)    
    el_pos = nominal_pcws.sh_des_vel_pcw.get_value(t)
    el_vel = nominal_pcws.el_des_vel_pcw.get_value(t)

    if attempt:
        while t < data.des_time[-1]:
            start_loop = time.time()
            t += meas_dt
            if t > data.des_time[-1]:
                t = data.des_time[-1]
            
            # send commands to motor in radians
            meas_state = np.array([
                                [sh_pos],
                                [el_pos],
                                [sh_vel],                                
                                [el_vel],
                                ]) 

            u_tvlqr = tau_tvlqr(
                                nominal_pcws=nominal_pcws,
                                meas_states_array=meas_state,
                                time=t
            )
            u_ff = nominal_pcws.el_tau_tvlqr_pcw.get_value(t)

            motor_params = motor_test_parameters(test,u_tvlqr,u_ff)
            tau_cmd = np.clip(motor_params.tau_cmd,-tau_limit,tau_limit)

            (el_pos, 
                el_vel, 
                el_tau) = await send_rad_command(controller_obj=servo,
                                                pos=nominal_pcws.el_des_pos_pcw.get_value(t),
                                                vel=nominal_pcws.el_des_vel_pcw.get_value(t),
                                                tau=tau_cmd,
                                                tau_limit=tau_limit,
                                                kp_scale=motor_params.kp_scale,
                                                kd_scale=motor_params.kd_scale,
                                                watchdog_timeout = None )
            (sh_pos,
                sh_vel,
                _,
                _) = await state_estimation(pr=pr, 
                                            pos_el=el_pos, 
                                            vel_el=el_vel,
                                            imu_init=init_euler_xyz[0])
            # store measured data
            recordedData.elbow_meas_pos[index]=el_pos
            recordedData.elbow_meas_vel[index]=el_vel
            recordedData.elbow_meas_tau[index]=el_tau
            recordedData.elbow_cmd_tau[index]=motor_params.tau_cmd            
            recordedData.shoulder_meas_pos[index]=sh_pos
            recordedData.shoulder_meas_vel[index]=sh_vel
            # store desired data
            recordedData.elbow_des_pos[index]=nominal_pcws.el_des_pos_pcw.get_value(t)
            recordedData.elbow_des_vel[index]=nominal_pcws.el_des_vel_pcw.get_value(t)
            recordedData.elbow_des_tau[index]=nominal_pcws.el_des_tau_pcw.get_value(t)
            recordedData.shoulder_des_pos[index]=nominal_pcws.sh_des_pos_pcw.get_value(t)
            recordedData.shoulder_des_vel[index]=nominal_pcws.sh_des_vel_pcw.get_value(t)
            recordedData.elbow_des_tau_tvlqr[index]=nominal_pcws.el_tau_tvlqr_pcw.get_value(t)
            recordedData.meas_time[index] = t            
            index += 1
            meas_dt = time.time() - start_loop
        
        #print("sleeping to let ZL finish")
        #time.sleep(0.1) #pause before checking location and beginning kickback  
        
        '''(el_pos, 
            el_vel, 
            el_tau) = await send_rad_command(controller_obj=servo,
                                            pos=0,
                                            vel=0,
                                            tau=0, #do not want to send a torque
                                            tau_limit=0,
                                            kp_scale=0,
                                            kd_scale=0,
                                            watchdog_timeout = 0.1 )

        print("elbow measured pos", el_pos)

        if el_pos < -1.9: #if at the bar when ZL finished
            success = True
            print("stopping motor in order to restart control, index is: ", index)
            await servo.set_stop()
            print("ZL success, success set to: ", str(success))
            recordedData.truncate(index)
            finalTime = t + 1 #include any pauses done
        else:
            print("ZL fail, stopping motor")
            await servo.set_stop()
            print("ZL fail, do not attempt, sucess set to : ", success)'''
            
    recordedData.truncate(index)
    finalTime = t
    
    return True, recordedData, finalTime, index

async def KickbackMotion(attempt, n, torque, tau_limit_KB, servo, init_euler_xyz, pr, init_time):

    success = False
    finalTime = init_time
    el_vel = 94305
    data_kickback = RecordData(n)

    if attempt:

        print("stopping motor in order to restart control")
        await servo.set_stop()    

        print("sleeping to let operator get ready")
        time.sleep(1) #pause before beginning kickback  

        # kickback running loop variables reset    
        index_KB = 0
        meas_dt = 0
        t = 0

        #max for which motor will run
        motorLoops = 49        

        #obtain starting velocity (should be zero)        
        (el_pos, 
            el_vel, 
            el_tau) = await send_rad_command(controller_obj=servo,
                                            pos=0,
                                            vel=0,
                                            tau=0, #do not want to send a torque
                                            tau_limit=0,
                                            kp_scale=0,
                                            kd_scale=0,
                                            watchdog_timeout = float('nan') ) #number 'nan' to prevent needing to stop the motor before next motion command

        while t < 0.05 or el_vel < 1.3:
            start_loop = time.time()
            t += meas_dt             

            if index_KB > motorLoops: #safety feature, motor loops should never be exceeded
                #kill motor                                                
                await servo.set_stop() 
                torque = 0
                print("servo killed at loop number:", index_KB)
                print("success set to", success)
                #exit kickback loop and do not enter LR loop
                success = False
                el_vel = 94305 #impossible velocity value to indicate failure
                break            
            
            
            tau_cmd = np.clip(torque,-tau_limit_KB,tau_limit_KB)
            
            (el_pos, 
                el_vel,
                el_tau) = await send_rad_command(controller_obj=servo,
                                                pos=0.0, #although 0, kp = 0 gives pos no affect
                                                vel=0.0, #although 0, kd = 0 gives vel no affect
                                                tau=tau_cmd,
                                                tau_limit= tau_limit_KB,
                                                kp_scale=0.0,
                                                kd_scale=0.0,
                                                watchdog_timeout = float("nan") )       
            
            (sh_pos,
                sh_vel,
                _,
                _) = await state_estimation(pr=pr, 
                                            pos_el=el_pos, 
                                            vel_el=el_vel, 
                                            imu_init=init_euler_xyz[0])

            # store measured data
            data_kickback.elbow_meas_pos[index_KB]=el_pos
            data_kickback.elbow_meas_vel[index_KB]=el_vel
            data_kickback.elbow_meas_tau[index_KB]=el_tau
            data_kickback.elbow_cmd_tau[index_KB]=torque            
            data_kickback.elbow_des_tau[index_KB]=tau_cmd
            data_kickback.shoulder_meas_pos[index_KB]=sh_pos
            data_kickback.shoulder_meas_vel[index_KB]=sh_vel
            data_kickback.meas_time[index_KB] = t + init_time + 1 # add any pauses to the current time
            
            index_KB += 1
            meas_dt = time.time() - start_loop

            finalTime = t + init_time + 1 # add any pauses to the current time

    if abs(el_vel) != 94305:
        success = True
        data_kickback.truncate(index_KB) 

    # Disabling the motor
    await servo.set_stop()
    return success, data_kickback, finalTime, index_KB

async def LeftToRightBarMotion(attempt, data_LR, nominal_pcws_LR, recordedData, test_LR, tau_limit, servo, init_euler_xyz, pr, init_time):

    finalTime = init_time
    
    # runnig loop variables    
    index_LR = 0
    t_LR = 0
    meas_dt_LR = 0
    
    # inital state array
    sh_pos_LR = nominal_pcws_LR.sh_des_pos_pcw.get_value(0)
    sh_vel_LR = nominal_pcws_LR.el_des_pos_pcw.get_value(0)    
    el_pos_LR = nominal_pcws_LR.sh_des_vel_pcw.get_value(0)
    el_vel_LR = nominal_pcws_LR.el_des_vel_pcw.get_value(0)

    if attempt:
        while t_LR < data_LR.des_time[-1]:

                start_loop = time.time()
                t_LR += meas_dt_LR

                if t_LR > data_LR.des_time[-1]:
                    t_LR = data_LR.des_time[-1]
                
                # send commands to motor in radians
                meas_state = np.array([
                                        [sh_pos_LR],
                                        [el_pos_LR],
                                        [sh_vel_LR],
                                        [el_vel_LR],
                                        ]) 

                u_tvlqr = tau_tvlqr(
                                        nominal_pcws=nominal_pcws_LR,
                                        meas_states_array=meas_state,
                                        time=t_LR
                )
                
                u_ff = nominal_pcws_LR.el_tau_tvlqr_pcw.get_value(t_LR)

                motor_params = motor_test_parameters(test_LR,u_tvlqr,u_ff)
                tau_cmd = np.clip(motor_params.tau_cmd,-tau_limit,tau_limit)

                (el_pos_LR, 
                el_vel_LR, 
                el_tau_LR) = await send_rad_command(controller_obj=servo,
                                                pos=nominal_pcws_LR.el_des_pos_pcw.get_value(t_LR),
                                                vel=nominal_pcws_LR.el_des_vel_pcw.get_value(t_LR),
                                                tau=tau_cmd,
                                                tau_limit=tau_limit,
                                                kp_scale=motor_params.kp_scale,
                                                kd_scale=motor_params.kd_scale,
                                                watchdog_timeout = float("nan") )               
                (sh_pos_LR,
                sh_vel_LR,
                _,
                _) = await state_estimation(pr=pr, 
                                                pos_el=el_pos_LR, 
                                                vel_el=el_vel_LR, 
                                                imu_init=init_euler_xyz[0])
                # store measured data
                recordedData.elbow_meas_pos[index_LR]=el_pos_LR
                recordedData.elbow_meas_vel[index_LR]=el_vel_LR
                recordedData.elbow_meas_tau[index_LR]=el_tau_LR
                recordedData.elbow_cmd_tau[index_LR]=motor_params.tau_cmd                
                recordedData.shoulder_meas_pos[index_LR]=sh_pos_LR
                recordedData.shoulder_meas_vel[index_LR]=sh_vel_LR
                # store desired data_LR
                recordedData.elbow_des_pos[index_LR]=nominal_pcws_LR.el_des_pos_pcw.get_value(t_LR)
                recordedData.elbow_des_vel[index_LR]=nominal_pcws_LR.el_des_vel_pcw.get_value(t_LR)
                recordedData.elbow_des_tau[index_LR]=nominal_pcws_LR.el_des_tau_pcw.get_value(t_LR)
                recordedData.shoulder_des_pos[index_LR]=nominal_pcws_LR.sh_des_pos_pcw.get_value(t_LR)
                recordedData.shoulder_des_vel[index_LR]=nominal_pcws_LR.sh_des_vel_pcw.get_value(t_LR)
                recordedData.elbow_des_tau_tvlqr[index_LR]=nominal_pcws_LR.el_tau_tvlqr_pcw.get_value(t_LR)
                recordedData.meas_time[index_LR] = t_LR + init_time         
                index_LR += 1
                meas_dt_LR = time.time() - start_loop

        #print("sleeping to let LR finish")
        #time.sleep(0.2) #pause before checking location and beginning kickback

    print("stopping motor in order to restart control")
    await servo.set_stop() 

    recordedData.truncate(index_LR)
    finalTime = t_LR + init_time + 0.0 #add any pauses taken

    return recordedData, finalTime, index_LR


async def RightToLeftBarRealTimeControl(attempt, data_LR, nominal_pcws_LR, recordedData, test_LR, tau_limit, servo,
                                        init_euler_xyz, pr, init_time, controller):
    finalTime = init_time

    # runnig loop variables
    index_LR = 0
    t_LR = 0
    meas_dt_LR = 0

    # inital state array
    sh_pos_LR = nominal_pcws_LR.sh_des_pos_pcw.get_value(0)
    sh_vel_LR = nominal_pcws_LR.el_des_pos_pcw.get_value(0)
    el_pos_LR = -1.0 * nominal_pcws_LR.sh_des_vel_pcw.get_value(0)
    el_vel_LR = -1.0 * nominal_pcws_LR.el_des_vel_pcw.get_value(0)

    if attempt:
        while t_LR < data_LR.des_time[-1]:

            start_loop = time.time()
            t_LR += meas_dt_LR

            if t_LR > data_LR.des_time[-1]:
                t_LR = data_LR.des_time[-1]

            # send commands to motor in radians
            if index_LR == 0:
                meas_state = np.array([
                                        [sh_pos_LR],
                                        [-1.0 * el_pos_LR],
                                        [sh_vel_LR],                                    
                                        [-1.0 * el_vel_LR]
                                        ])
            else:
                meas_state = np.array([
                                        [sh_pos_LR],
                                        [-1.0 * el_pos_LR],
                                        [sh_vel_LR],                                    
                                        [-1.0 * el_vel_LR]
                                        ])
                                    
            if index_LR < 10:
                print(meas_state)


            tau_cmd = -1.0 * np.clip(1.0*controller.get_torque(meas_state), -tau_limit, tau_limit)

            (el_pos_LR,
             el_vel_LR,
             el_tau_LR) = await send_rad_command(controller_obj=servo,
                                                 pos=0.0,
                                                 vel=0.0,
                                                 tau=tau_cmd,
                                                 tau_limit=tau_limit,
                                                 kp_scale=0.0,
                                                 kd_scale=0.0,
                                                 watchdog_timeout=float("nan"))
                                                 
            sh_pos_LR,sh_vel_LR = await state_estimation_v2(pr)
            
            # store measured data
            recordedData.elbow_meas_pos[index_LR] = el_pos_LR
            recordedData.elbow_meas_vel[index_LR] = el_vel_LR
            recordedData.elbow_meas_tau[index_LR] = el_tau_LR
            recordedData.elbow_cmd_tau[index_LR] = 1.0*controller.get_torque(meas_state)            
            recordedData.shoulder_meas_pos[index_LR] = -1 * sh_pos_LR
            recordedData.shoulder_meas_vel[index_LR] = -1 * sh_vel_LR
            # store desired data_LR
            recordedData.elbow_des_pos[index_LR] = -1 * nominal_pcws_LR.el_des_pos_pcw.get_value(t_LR)
            recordedData.elbow_des_vel[index_LR] = -1 * nominal_pcws_LR.el_des_vel_pcw.get_value(t_LR)
            recordedData.elbow_des_tau[index_LR] = -1 * nominal_pcws_LR.el_des_tau_pcw.get_value(t_LR)
            recordedData.shoulder_des_pos[index_LR] = -1 * nominal_pcws_LR.sh_des_pos_pcw.get_value(t_LR)
            recordedData.shoulder_des_vel[index_LR] = -1 * nominal_pcws_LR.sh_des_vel_pcw.get_value(t_LR)
            recordedData.elbow_des_tau_tvlqr[index_LR] = -1 * nominal_pcws_LR.el_tau_tvlqr_pcw.get_value(t_LR)
            recordedData.meas_time[index_LR] = t_LR + init_time
            index_LR += 1
            meas_dt_LR = time.time() - start_loop

        print("sleeping to let LR2 finish")
        time.sleep(0.2) #pause before checking location and beginning kickback
 

    print("stopping motor in order to restart control")
    await servo.set_stop() 


    recordedData.truncate(index_LR)
    finalTime = t_LR + init_time + 0.2  # add any pauses taken

    return recordedData, finalTime, index_LR
    
async def LeftToRightBarRealTimeControl(attempt, data_LR, nominal_pcws_LR, recordedData, test_LR, tau_limit, servo,
                                        init_euler_xyz, pr, init_time, controller):
    finalTime = init_time

    # runnig loop variables
    index_LR = 0
    t_LR = 0
    meas_dt_LR = 0

    # inital state array
    sh_pos_LR = nominal_pcws_LR.sh_des_pos_pcw.get_value(0)
    sh_vel_LR = nominal_pcws_LR.el_des_pos_pcw.get_value(0)
    el_pos_LR = nominal_pcws_LR.sh_des_vel_pcw.get_value(0)
    el_vel_LR = nominal_pcws_LR.el_des_vel_pcw.get_value(0)

    if attempt:
        while t_LR < data_LR.des_time[-1]:

            start_loop = time.time()
            t_LR += meas_dt_LR

            if t_LR > data_LR.des_time[-1]:
                t_LR = data_LR.des_time[-1]

            # send commands to motor in radians
            meas_state = np.array([
                [sh_pos_LR],
                [el_pos_LR],
                [sh_vel_LR],
                [el_vel_LR],
            ])
            
            if index_LR < 10:
                print(meas_state)


            tau_cmd = np.clip(1.00*controller.get_torque(meas_state), -tau_limit, tau_limit)

            (el_pos_LR,
             el_vel_LR,
             el_tau_LR) = await send_rad_command(controller_obj=servo,
                                                 pos=0.0,
                                                 vel=0.0,
                                                 tau=tau_cmd,
                                                 tau_limit=tau_limit,
                                                 kp_scale=0.0,
                                                 kd_scale=0.0,
                                                 watchdog_timeout=float("nan"))
            (sh_pos_LR,
             sh_vel_LR,
             _,
             _) = await state_estimation(pr=pr,
                                         pos_el=el_pos_LR,
                                         vel_el=el_vel_LR,
                                         imu_init=init_euler_xyz[0])
            # store measured data
            recordedData.elbow_meas_pos[index_LR] = el_pos_LR
            recordedData.elbow_meas_vel[index_LR] = el_vel_LR
            recordedData.elbow_meas_tau[index_LR] = el_tau_LR
            recordedData.elbow_cmd_tau[index_LR] = 1.00*controller.get_torque(meas_state)            
            recordedData.shoulder_meas_pos[index_LR] = sh_pos_LR
            recordedData.shoulder_meas_vel[index_LR] = sh_vel_LR
            # store desired data_LR
            recordedData.elbow_des_pos[index_LR] = nominal_pcws_LR.el_des_pos_pcw.get_value(t_LR)
            recordedData.elbow_des_vel[index_LR] = nominal_pcws_LR.el_des_vel_pcw.get_value(t_LR)
            recordedData.elbow_des_tau[index_LR] = nominal_pcws_LR.el_des_tau_pcw.get_value(t_LR)
            recordedData.shoulder_des_pos[index_LR] = nominal_pcws_LR.sh_des_pos_pcw.get_value(t_LR)
            recordedData.shoulder_des_vel[index_LR] = nominal_pcws_LR.sh_des_vel_pcw.get_value(t_LR)
            recordedData.elbow_des_tau_tvlqr[index_LR] = nominal_pcws_LR.el_tau_tvlqr_pcw.get_value(t_LR)
            recordedData.meas_time[index_LR] = t_LR + init_time
            index_LR += 1
            meas_dt_LR = time.time() - start_loop

        # print("sleeping to let LR finish")
        # time.sleep(0.2) #pause before checking location and beginning kickback

    print("stopping motor in order to restart control")
    await servo.set_stop()

    recordedData.truncate(index_LR)
    finalTime = t_LR + init_time + 0.0  # add any pauses taken

    return recordedData, finalTime, index_LR

async def CatchMotion(attempt, n_catch, torque_CATCH, tau_limit_CATCH, servo, init_euler_xyz, pr, init_time):

    success = False
    finalTime = init_time
    data_CATCH = RecordData(n_catch)

    index_CATCH = 0
    t_CATCH = 0
    meas_dt_CATCH = 0

    print("starting catch: ", torque_CATCH)

    if attempt:
        while index_CATCH < n_catch: #t < 3.65 or el_vel < 1.5: #do not break for time instead take data for n loops
            start_loop = time.time()
            t_CATCH += meas_dt_CATCH			           
            
            tau_cmd_CATCH = np.clip(torque_CATCH,-tau_limit_CATCH,tau_limit_CATCH)
            #print(tau_cmd_CATCH)
            
            (el_pos, 
                el_vel, 
                el_tau) = await send_rad_command(controller_obj=servo,
                                                pos=0.0, #although 0, kp = 0 gives pos no affect
                                                vel=0.0, #although 0, kd = 0 gives vel no affect
                                                tau=tau_cmd_CATCH,
                                                tau_limit=tau_limit_CATCH,
                                                kp_scale=0.0,
                                                kd_scale=0.0,
                                                watchdog_timeout = 0.05)       
            
            (sh_pos,
                sh_vel,
                _,
                _) = await state_estimation(pr=pr, 
                                            pos_el=el_pos, 
                                            vel_el=el_vel, 
                                            imu_init=init_euler_xyz[0])
            
            _, _, _, index_euler_xyz = await read_imu_data(pr)

            # store measured data
            data_CATCH.elbow_meas_pos[index_CATCH]=el_pos
            data_CATCH.elbow_meas_vel[index_CATCH]=el_vel
            data_CATCH.elbow_meas_tau[index_CATCH]=el_tau
            data_CATCH.elbow_cmd_tau[index_CATCH]=torque_CATCH
            data_CATCH.elbow_des_tau[index_CATCH]=tau_cmd_CATCH
            data_CATCH.shoulder_meas_pos[index_CATCH]=sh_pos
            data_CATCH.shoulder_meas_vel[index_CATCH]=sh_vel
            data_CATCH.meas_time[index_CATCH] = t_CATCH + init_time
            
            index_CATCH += 1
            meas_dt_CATCH = time.time() - start_loop

        if el_pos > 1.9: #if at the bar when CATCH finished
            success = True
            print("stopping motor in order to restart control, index is: ", index_CATCH)
            await servo.set_stop()
            print("Catch success, success set to: ", str(success))
            data_CATCH.truncate(index_CATCH)
            finalTime = t_CATCH + init_time #include any pauses done
        else:
            print("Catch fail, stopping motor")
            await servo.set_stop()
            print("Catch fail, do not attempt, sucess set to : ", success)

    return success, data_CATCH, finalTime, index_CATCH

async def SubequentKickbackMotion(attempt, n_KB2, torque_KB2, tau_limit_KB2, servo, pr, init_time):

    success = False
    finalTime = init_time
    el_vel = 94305
    data_KB2 = RecordData(n_KB2)

    if attempt:

        print("stopping motor in order to restart control")
        await servo.set_stop()    

        # runnig loop variables    
        index_KB2 = 0
        t_KB2 = 0
        meas_dt_KB2 = 0

        #length for which motor will run
        motorLoops_KB2 = 69 #WARNING 30

        print("sleeping to let operator get ready")
        time.sleep(1)
  
        #obtain starting velocity (should be zero)        
        (el_pos, 
            el_vel, 
            el_tau) = await send_rad_command(controller_obj=servo,
                                            pos=0,
                                            vel=0,
                                            tau=0, #do not want to send a torque
                                            tau_limit=0,
                                            kp_scale=0,
                                            kd_scale=0,
                                            watchdog_timeout = float('nan') ) #number 'nan' to prevent needing to stop the motor before next motion command

        while t_KB2 < 0.05 or el_vel > -1.3: #t_KB2 < 0.07 or el_vel > -2.0
            start_loop = time.time()
            t_KB2 += meas_dt_KB2
            
            if index_KB2 > motorLoops_KB2:
                #kill motor                                                
                await servo.set_stop() 
                torque = 0
                print("servo killed at loop number:", index_KB2)
                print("success set to", success)
                #exit kickback loop and do not enter LR loop
                success = False
                el_vel = 94305 #impossible velocity value to indicate failure
                break
            
            #print(index_KB2)
            
            tau_cmd_KB2 = np.clip(torque_KB2,-tau_limit_KB2,tau_limit_KB2)
            #print(tau_cmd_KB2)
            (el_pos, 
             el_vel, 
             el_tau) = await send_rad_command(controller_obj=servo,
                                              pos=0.0, #although 0, kp = 0 gives pos no affect
                                              vel=0.0, #although 0, kd = 0 gives vel no affect
                                              tau=tau_cmd_KB2,
                                              tau_limit=tau_limit_KB2,
                                              kp_scale=0.0,
                                              kd_scale=0.0)
                                              #no watchdog written therefore set to default motor value        

            sh_pos,sh_vel = await state_estimation_v2(pr)

            # store measured data
            data_KB2.elbow_meas_pos[index_KB2]=el_pos
            data_KB2.elbow_meas_vel[index_KB2]=el_vel
            data_KB2.elbow_meas_tau[index_KB2]=el_tau
            data_KB2.elbow_cmd_tau[index_KB2]=torque_KB2
            data_KB2.elbow_des_tau[index_KB2]=tau_cmd_KB2
            data_KB2.shoulder_meas_pos[index_KB2]=sh_pos
            data_KB2.shoulder_meas_vel[index_KB2]=sh_vel
            data_KB2.meas_time[index_KB2] = t_KB2 + init_time + 1 # add any pauses to the current time
            
            index_KB2 += 1
            meas_dt_KB2 = time.time() - start_loop
            
            finalTime = t_KB2 + init_time + 1

    if abs(el_vel) != 94305:
        success = True 
        data_KB2.truncate(index_KB2)
        finalTime = t_KB2 + init_time + 1

    # Disabling the motor
    await servo.set_stop()
    return success, data_KB2, finalTime, index_KB2

async def SubsequentLeftToRightBarMotion(attempt, data_LR2, nominal_pcws_LR2, recordedData, test_LR, tau_limit, servo, pr, init_time):

    finalTime = init_time

    if attempt:

        # runnig loop variables    
        index_LR2 = 0
        t_LR2 = 0
        meas_dt_LR2 = 0

        # inital state array
        sh_pos_LR = nominal_pcws_LR2.sh_des_pos_pcw.get_value(0)
        el_pos_LR = nominal_pcws_LR2.el_des_pos_pcw.get_value(0)    
        sh_vel_LR = nominal_pcws_LR2.sh_des_vel_pcw.get_value(0)
        el_vel_LR = nominal_pcws_LR2.el_des_vel_pcw.get_value(0)

        while t_LR2 < data_LR2.des_time[-1]:

            start_loop = time.time()
            t_LR2 += meas_dt_LR2

            if t_LR2 > data_LR2.des_time[-1]:
                t_LR2 = data_LR2.des_time[-1]
            
            # send commands to motor in radians
            meas_state = np.array([
                                    [-1 * sh_pos_LR],
                                    [el_pos_LR],
                                    [-1 * sh_vel_LR],                                    
                                    [el_vel_LR]
                                    ])
            
            '''meas_state = np.array([
                                    [sh_pos_LR],
                                    [el_pos_LR],
                                    [sh_vel_LR],                                    
                                    [el_vel_LR]
                                    ])'''

            u_tvlqr = tau_tvlqr_subsequent(
                                    nominal_pcws=nominal_pcws_LR2,
                                    meas_states_array=meas_state,
                                    time=t_LR2
            )
            
            u_ff = nominal_pcws_LR2.el_tau_tvlqr_pcw.get_value(t_LR2)

            motor_params = motor_test_parameters(test_LR,u_tvlqr,u_ff)
            tau_cmd = np.clip(motor_params.tau_cmd,-tau_limit,tau_limit)

            (el_pos_LR, 
            el_vel_LR, 
            el_tau_LR) = await send_rad_command(controller_obj=servo,
                                            pos=-nominal_pcws_LR2.el_des_pos_pcw.get_value(t_LR2),
                                            vel=-nominal_pcws_LR2.el_des_vel_pcw.get_value(t_LR2),
                                            tau=tau_cmd, #fix me to be negative for TVLQR
                                            tau_limit=tau_limit,
                                            kp_scale=motor_params.kp_scale,
                                            kd_scale=motor_params.kd_scale,
                                            watchdog_timeout = 0.05 )    
           
            sh_pos_LR,sh_vel_LR = await state_estimation_v2(pr)

            # store measured data
            recordedData.elbow_meas_pos[index_LR2]=el_pos_LR
            recordedData.elbow_meas_vel[index_LR2]=el_vel_LR
            recordedData.elbow_meas_tau[index_LR2]=el_tau_LR
            recordedData.elbow_cmd_tau[index_LR2]=motor_params.tau_cmd
            recordedData.shoulder_meas_pos[index_LR2]= -1 * sh_pos_LR
            recordedData.shoulder_meas_vel[index_LR2]= -1 * sh_vel_LR
            # store desired data_LR
            recordedData.elbow_des_pos[index_LR2]= -1 * nominal_pcws_LR2.el_des_pos_pcw.get_value(t_LR2)
            recordedData.elbow_des_vel[index_LR2]= -1 * nominal_pcws_LR2.el_des_vel_pcw.get_value(t_LR2)
            recordedData.elbow_des_tau[index_LR2]= -1 * nominal_pcws_LR2.el_des_tau_pcw.get_value(t_LR2)
            recordedData.shoulder_des_pos[index_LR2]= -1 * nominal_pcws_LR2.sh_des_pos_pcw.get_value(t_LR2)
            recordedData.shoulder_des_vel[index_LR2]= -1 * nominal_pcws_LR2.sh_des_vel_pcw.get_value(t_LR2)
            recordedData.elbow_des_tau_tvlqr[index_LR2]= -1 * nominal_pcws_LR2.el_tau_tvlqr_pcw.get_value(t_LR2)
            recordedData.meas_time[index_LR2] = t_LR2 + init_time
            index_LR2 += 1
            meas_dt_LR2 = time.time() - start_loop

        print("sleeping to let LR2 finish")
        time.sleep(0.2) #pause before checking location and beginning kickback        

    print("stopping motor in order to restart control")
    await servo.set_stop() 

    recordedData.truncate(index_LR2)
    finalTime = t_LR2 + init_time + 0.2 #add any pauses taken

    return recordedData, finalTime, index_LR2

async def SubsequentCatchMotion(attempt, n_catch, torque_CATCH, tau_limit_CATCH, servo, pr, init_time):

    success = False
    finalTime = init_time
    data_CATCH = RecordData(n_catch)

    index_CATCH = 0
    t_CATCH = 0
    meas_dt_CATCH = 0

    print("starting catch: ", torque_CATCH)

    if attempt:
        while index_CATCH < n_catch: #t < 3.65 or el_vel < 1.5: #do not break for time instead take data for n loops
            start_loop = time.time()
            t_CATCH += meas_dt_CATCH			           
            
            tau_cmd_CATCH = np.clip(torque_CATCH,-tau_limit_CATCH,tau_limit_CATCH)
            #print(tau_cmd_CATCH)
            
            (el_pos, 
                el_vel, 
                el_tau) = await send_rad_command(controller_obj=servo,
                                                pos=0.0, #although 0, kp = 0 gives pos no affect
                                                vel=0.0, #although 0, kd = 0 gives vel no affect
                                                tau=tau_cmd_CATCH,
                                                tau_limit=tau_limit_CATCH,
                                                kp_scale=0.0,
                                                kd_scale=0.0,
                                                watchdog_timeout = 0.05)       
            

            sh_pos,sh_vel = await state_estimation_v2(pr)

            # store measured data
            data_CATCH.elbow_meas_pos[index_CATCH]=el_pos
            data_CATCH.elbow_meas_vel[index_CATCH]=el_vel
            data_CATCH.elbow_meas_tau[index_CATCH]=el_tau
            data_CATCH.elbow_cmd_tau[index_CATCH]=torque_CATCH            
            data_CATCH.elbow_des_tau[index_CATCH]=tau_cmd_CATCH
            data_CATCH.shoulder_meas_pos[index_CATCH]=sh_pos
            data_CATCH.shoulder_meas_vel[index_CATCH]=sh_vel
            data_CATCH.meas_time[index_CATCH] = t_CATCH + init_time
            
            index_CATCH += 1
            meas_dt_CATCH = time.time() - start_loop

        if el_pos < -1.9: #if at the bar when CATCH finished
            success = True
            print("stopping motor in order to restart control, index is: ", index_CATCH)
            await servo.set_stop()
            print("Catch success, success set to: ", str(success))
            data_CATCH.truncate(index_CATCH)
            finalTime = t_CATCH + init_time #include any pauses done
        else:
            print("Catch fail, stopping motor")
            await servo.set_stop()
            print("Catch fail, do not attempt, sucess set to : ", success)

    return success, data_CATCH, finalTime, index_CATCH

async def KickbackMotionRL(attempt, n, torque_pop, torque_KB, tau_limit_KB, servo, init_euler_xyz, pr, init_time):

    success = False
    finalTime = init_time
    data_kickback = RecordData(n)

    if attempt:

        print("stopping motor in order to restart control")
        await servo.set_stop()    

        #print("sleeping to let operator get ready")
        #time.sleep(2) #pause before beginning kickback  

        # kickback running loop variables reset    
        index_KB = 0
        meas_dt = 0
        t = 0

        #length for which motor will run
        #popLoops = 25
        popVel = -2.2 #2.4
        pop = True
        KBLoops = 60 #WARNING #49 for the hook 
        releaseVel = 5.0
        releasePos = 2.21
        released = False      

        #obtain starting velocity (should be zero)        
        (el_pos, 
            el_vel, 
            el_tau) = await send_rad_command(controller_obj=servo,
                                            pos=0,
                                            vel=0,
                                            tau=0, #do not want to send a torque
                                            tau_limit=0,
                                            kp_scale=0,
                                            kd_scale=0,
                                            watchdog_timeout = float('nan') ) #number 'nan' to prevent needing to stop the motor before next motion command
       
        el_pos_init = el_pos

        while index_KB < n: #t < 0.05 or el_vel < 1.3:
            start_loop = time.time()
            t += meas_dt             

            if t < 0.04: # el_pos - el_pos_init < 2*np.pi/180 and pop: #0.05
                torque = -3.5 
            elif el_vel > popVel and pop:#pop state
                torque = torque_pop 
            elif el_vel < popVel and pop: #transition state
                torque = torque_KB
                pop = False
            elif not pop and el_pos < releasePos: #el_vel < releaseVel:# t < 0.285: #KB state
                #for data collection, instead of break statement 
                if released:
                        torque = 0
                else:  
                        torque = torque_KB 
            elif not pop and el_pos > releasePos: #el_vel > releaseVel:# t < 0.285: #KB state
                torque = 0
                released = True  
                break  #for data collection, remove break statement                                   
            else: #end KB transition
                #kill motor                                                
                await servo.set_stop() 
                torque = 0
                '''print("servo killed at loop number:", index_KB)
                print("success set to", success)
                #exit kickback loop and do not enter LR loop
                success = False
                el_vel = 94305 #impossible velocity value to indicate failure
                break '''           
            
            
            tau_cmd = np.clip(torque,-tau_limit_KB,tau_limit_KB)
            
            (el_pos, 
                el_vel,
                el_tau) = await send_rad_command(controller_obj=servo,
                                                pos=0.0, #although 0, kp = 0 gives pos no affect
                                                vel=0.0, #although 0, kd = 0 gives vel no affect
                                                tau=tau_cmd,
                                                tau_limit= tau_limit_KB,
                                                kp_scale=0.0,
                                                kd_scale=0.0,
                                                watchdog_timeout = float("nan") )       
            
            (sh_pos,
                sh_vel,
                _,
                _) = await state_estimation(pr=pr, 
                                            pos_el=el_pos, 
                                            vel_el=el_vel, 
                                            imu_init=init_euler_xyz[0])
            
            #sh_pos,sh_vel = await state_estimation_v2(pr)

            # store measured data
            data_kickback.elbow_meas_pos[index_KB]=el_pos
            data_kickback.elbow_meas_vel[index_KB]=el_vel
            data_kickback.elbow_meas_tau[index_KB]=el_tau
            data_kickback.elbow_cmd_tau[index_KB]=torque
            data_kickback.elbow_des_tau[index_KB]=tau_cmd
            data_kickback.shoulder_meas_pos[index_KB]=sh_pos
            data_kickback.shoulder_meas_vel[index_KB]=sh_vel
            data_kickback.meas_time[index_KB] = t + init_time# + 2 # add any pauses to the current time
            
            index_KB += 1
            meas_dt = time.time() - start_loop

            finalTime = t + init_time# + 2 # add any pauses to the current time

    if index_KB < n - 5:
        success = True
        data_kickback.truncate(index_KB) 

    # Disabling the motor
    await servo.set_stop()
    return success, data_kickback, finalTime, index_KB

async def RightToLeftBarMotion(attempt, data_RL, nominal_pcws_RL, recordedData, test_RL, tau_limit, servo, init_euler_xyz, pr, init_time):

    finalTime = init_time
    
    # runnig loop variables    
    index_RL = 0
    t_RL = 0
    meas_dt_RL = 0
    
    # inital state array
    sh_pos_RL = nominal_pcws_RL.sh_des_pos_pcw.get_value(0)
    sh_vel_RL = nominal_pcws_RL.el_des_pos_pcw.get_value(0)    
    el_pos_RL = nominal_pcws_RL.sh_des_vel_pcw.get_value(0)
    el_vel_RL = nominal_pcws_RL.el_des_vel_pcw.get_value(0)

    if attempt:
        while t_RL < data_RL.des_time[-1]:

                start_loop = time.time()
                t_RL += meas_dt_RL

                if t_RL > data_RL.des_time[-1]:
                    t_RL = data_RL.des_time[-1]
                
                # send commands to motor in radians
                meas_state = np.array([
                                        [sh_pos_RL],
                                        [el_pos_RL],
                                        [sh_vel_RL],
                                        [el_vel_RL],
                                        ]) 

                u_tvlqr = tau_tvlqr(
                                        nominal_pcws=nominal_pcws_RL,
                                        meas_states_array=meas_state,
                                        time=t_RL
                )
                
                u_ff = nominal_pcws_RL.el_tau_tvlqr_pcw.get_value(t_RL)

                motor_params = motor_test_parameters(test_RL,u_tvlqr,u_ff)
                tau_cmd = np.clip(motor_params.tau_cmd,-tau_limit,tau_limit)

                (el_pos_RL, 
                el_vel_RL, 
                el_tau_RL) = await send_rad_command(controller_obj=servo,
                                                pos=nominal_pcws_RL.el_des_pos_pcw.get_value(t_RL),
                                                vel=nominal_pcws_RL.el_des_vel_pcw.get_value(t_RL),
                                                tau=tau_cmd,
                                                tau_limit=tau_limit,
                                                kp_scale=motor_params.kp_scale,
                                                kd_scale=motor_params.kd_scale,
                                                watchdog_timeout = 0.1)               
                (sh_pos_RL,
                sh_vel_RL,
                _,
                _) = await state_estimation(pr=pr, 
                                                pos_el=el_pos_RL, 
                                                vel_el=el_vel_RL, 
                                                imu_init=init_euler_xyz[0])
                # store measured data
                recordedData.elbow_meas_pos[index_RL]=el_pos_RL
                recordedData.elbow_meas_vel[index_RL]=el_vel_RL
                recordedData.elbow_meas_tau[index_RL]=el_tau_RL
                recordedData.elbow_cmd_tau[index_RL]=motor_params.tau_cmd
                recordedData.shoulder_meas_pos[index_RL]=sh_pos_RL
                recordedData.shoulder_meas_vel[index_RL]=sh_vel_RL
                # store desired data_RL
                recordedData.elbow_des_pos[index_RL]=nominal_pcws_RL.el_des_pos_pcw.get_value(t_RL)
                recordedData.elbow_des_vel[index_RL]=nominal_pcws_RL.el_des_vel_pcw.get_value(t_RL)
                recordedData.elbow_des_tau[index_RL]=nominal_pcws_RL.el_des_tau_pcw.get_value(t_RL)
                recordedData.shoulder_des_pos[index_RL]=nominal_pcws_RL.sh_des_pos_pcw.get_value(t_RL)
                recordedData.shoulder_des_vel[index_RL]=nominal_pcws_RL.sh_des_vel_pcw.get_value(t_RL)
                recordedData.elbow_des_tau_tvlqr[index_RL]=nominal_pcws_RL.el_tau_tvlqr_pcw.get_value(t_RL)
                recordedData.meas_time[index_RL] = t_RL + init_time         
                index_RL += 1
                meas_dt_RL = time.time() - start_loop

        print("sleeping to let LR finish")
        time.sleep(0.2) #pause before checking location and beginning kickback

    print("stopping motor in order to restart control")
    await servo.set_stop() 

    recordedData.truncate(index_RL)
    finalTime = t_RL + init_time + 0.2 #add any pauses taken

    return recordedData, finalTime, index_RL
