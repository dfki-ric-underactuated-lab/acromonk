import asyncio
import numpy as np
import moteus
import moteus_pi3hat
import time

from datetime import datetime
import os
import matplotlib.pyplot as plt
from collections import namedtuple

# Local imports
from utils import (send_rad_command,
                   read_imu_data,
                   state_estimation,
                   save_data,
                   read_data,
                   zero_offset,
                   create_nominal_pcws,
                   tau_tvlqr,   
                   motor_test_parameters,
                   generate_path,
                   plot_data,
                   state_estimation_v2,
                   plot_custom_data,
                   prepare_data)

from states import (ZeroToLeftBarMotion,
                    KickbackMotion,
                    LeftToRightBarMotion,
                    SupportArmUpIMUStateInit,
                    LeadArmUpIMUStateInit,
                    CatchMotion,
                    SubequentKickbackMotion,
                    SubsequentLeftToRightBarMotion,
                    SubsequentCatchMotion,
                    KickbackMotionRL,
                    RightToLeftBarMotion,                    
                    RecordData)

def convert_to_NamedTuple(recordedData):

    n = len(recordedData.des_time)
    des_time=recordedData.des_time 
    shoulder_des_pos=recordedData.shoulder_des_pos 
    shoulder_des_vel=recordedData.shoulder_des_vel 
    shoulder_des_acc=recordedData.shoulder_des_acc 
    shoulder_des_jerk=recordedData.shoulder_des_jerk 
    shoulder_des_tau=recordedData.shoulder_des_tau 
    ## elbow.values
    elbow_des_pos=recordedData.elbow_des_pos 
    elbow_des_vel=recordedData.elbow_des_vel 
    elbow_des_acc=recordedData.elbow_des_acc 
    elbow_des_jerk=recordedData.elbow_des_jerk 
    elbow_des_tau=recordedData.elbow_des_tau 
    elbow_des_tau_tvlqr=recordedData.elbow_des_tau_tvlqr 
    # K values
    k1=recordedData.k1 
    k2=recordedData.k2 
    k3=recordedData.k3 
    k4=recordedData.k4 
    k0=recordedData.k0 
    # converting the desired trajectories according to the gear ratio
    shoulder_meas_pos=recordedData.shoulder_meas_pos 
    shoulder_meas_vel=recordedData.shoulder_meas_vel 
    elbow_meas_pos=recordedData.elbow_meas_pos 
    elbow_meas_vel=recordedData.elbow_meas_vel 
    elbow_meas_tau=recordedData.elbow_meas_tau
    meas_time=recordedData.meas_time 
    dt = recordedData.meas_time[2] - recordedData.meas_time[1]

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
                dt=dt
                )
    return data

def combine_data(dataPre, dataPost):

    print("len pre: ", len(dataPre.des_time))
    print("len post: ", len(dataPost.des_time))

    dataPre.des_time=np.hstack((dataPre.des_time,dataPost.des_time))
    dataPre.shoulder_des_pos=np.hstack((dataPre.shoulder_des_pos,dataPost.shoulder_des_pos))  
    dataPre.shoulder_des_vel=np.hstack((dataPre.shoulder_des_vel,dataPost.shoulder_des_vel))  
    dataPre.shoulder_des_acc=np.hstack((dataPre.shoulder_des_acc,dataPost.shoulder_des_acc))  
    dataPre.shoulder_des_jerk=np.hstack((dataPre.shoulder_des_jerk,dataPost.shoulder_des_jerk))  
    dataPre.shoulder_des_tau=np.hstack((dataPre.shoulder_des_tau,dataPost.shoulder_des_tau))  
    ## elbow.values
    dataPre.elbow_des_pos=np.hstack((dataPre.elbow_des_pos,dataPost.elbow_des_pos))  
    dataPre.elbow_des_vel=np.hstack((dataPre.elbow_des_vel,dataPost.elbow_des_vel))  
    dataPre.elbow_des_acc=np.hstack((dataPre.elbow_des_acc,dataPost.elbow_des_acc))  
    dataPre.elbow_des_jerk=np.hstack((dataPre.elbow_des_jerk,dataPost.elbow_des_jerk))  
    dataPre.elbow_des_tau=np.hstack((dataPre.elbow_des_tau,dataPost.elbow_des_tau))  
    dataPre.elbow_des_tau_tvlqr=np.hstack((dataPre.elbow_des_tau_tvlqr,dataPost.elbow_des_tau_tvlqr))  
    # K values
    dataPre.k1=np.hstack((dataPre.k1,dataPost.k1))  
    dataPre.k2=np.hstack((dataPre.k2,dataPost.k2))  
    dataPre.k3=np.hstack((dataPre.k3,dataPost.k3))  
    dataPre.k4=np.hstack((dataPre.k4,dataPost.k4))  
    dataPre.k0=np.hstack((dataPre.k0,dataPost.k0))  
    # converting the desired trajectories according to the gear ratio
    dataPre.shoulder_meas_pos=np.hstack((dataPre.shoulder_meas_pos,dataPost.shoulder_meas_pos))  
    dataPre.shoulder_meas_vel=np.hstack((dataPre.shoulder_meas_vel,dataPost.shoulder_meas_vel))  
    dataPre.elbow_meas_pos=np.hstack((dataPre.elbow_meas_pos,dataPost.elbow_meas_pos))  
    dataPre.elbow_meas_vel=np.hstack((dataPre.elbow_meas_vel,dataPost.elbow_meas_vel))  
    dataPre.elbow_meas_tau=np.hstack((dataPre.elbow_meas_tau,dataPost.elbow_meas_tau)) 
    dataPre.meas_time=np.hstack((dataPre.meas_time,dataPost.meas_time)) 

    print("len pre: ", len(dataPre.des_time))
    return dataPre

async def main(csv_data,csv_data_LR,csv_data_LR2,csv_data_RL,test,test_LR,test_RL):
    
    print("Motor Enabled.")
    # motor id and bus number mapping
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            bus_number: [motor_id],
        },
    )
    # create a moteus.Controller object for interaction with controller
    servo = moteus.Controller(id=motor_id, transport=transport)

    # Reset the IMU
    moteus_pi3hat.Pi3HatRouter(mounting_deg={'roll' : 20,
                                             'pitch': 0,
                                             'yaw'  : 0})  
    time.sleep(0.5)                                                             
    #initialize the IMU with USB up
    init_euler_xyz, pr = await SupportArmUpIMUStateInit(servo)  
    
    #pause for setting the acroMonk into starting RL position
    print("sleeping to let the operator set the Monk")
    time.sleep(10)
    print("sleeping 10 seconds over")

    attempt = True
    success = False

    try:    
        t = 0
        
        #uncomment only the data structures needed for the desired motion
        
        #prepare data for moteus controller, RL brachiation
        
        data_RL = prepare_data(csv_data_RL)    
        nominal_pcws_RL= create_nominal_pcws(data_RL)
        recordedData_RL = RecordData(csv_data=csv_data_RL)

        '''
        # prepare data for moteus controller, LR first brachiation
        data_LR1 = prepare_data(csv_data_LR)    
        nominal_pcws_LR1= create_nominal_pcws(data_LR1)
        recordedData_LR1 = RecordData(csv_data=csv_data_LR)
        
        # prepare data for moteus controller, LR second brachiation
        data_LR2 = prepare_data(csv_data_LR2)
        nominal_pcws_LR2= create_nominal_pcws(data_LR2)
        recordedData_LR2 = RecordData(csv_data=csv_data_LR2)
        
        #prepare data for moteus controller, , LR third brachiation 
        data_LR3 = prepare_data(csv_data_LR)   
        nominal_pcws_LR3= create_nominal_pcws(data_LR3)
        recordedData_LR3 = RecordData(csv_data=csv_data_LR)'''
        

        #include only the desired motions 
        '''
        success, data_KB, t, index_KB = await KickbackMotion(attempt, 65, 2.9, 3.0, servo, init_euler_xyz, pr, t)
        
        if success == False:
             attempt = False
        
        recorded_data_LR, t, index_LR = await LeftToRightBarMotion(attempt,data_LR1, nominal_pcws_LR1, recordedData_LR1, test_LR, 1.25, servo, init_euler_xyz, pr, t)    

        success, data_CATCH, t, index_CATCH = await CatchMotion(attempt, 20, 0.8, 1.2, servo, init_euler_xyz, pr, t)
        
        if success == False:
            attempt = False
        
        pr = await LeadArmUpIMUStateInit(servo)
        
        t = t + 2 #waited 1 second when resetting IMU        

        success, data_KB2, t, index_KB2 = await SubequentKickbackMotion(attempt, 65, -2.9, 3.0, servo, pr, t)

        if success == False:
            attempt = False
        
        recorded_data_LR2, t, index_LR2 = await SubsequentLeftToRightBarMotion(attempt,data_LR2, nominal_pcws_LR2, recordedData_LR2, test_LR, 1.25, servo, pr, t)
        
        success, data_CATCH2, t, index_CATCH2 = await SubsequentCatchMotion(attempt, 20, -0.8, 1.2, servo, pr, t)
        
        #initialize the IMU with USB up
        
        init_euler_xyz, pr = await SupportArmUpIMUStateInit(servo)
        
        t = t + 2 #waited 1 second when resetting IMU         
        
        success, data_KBx, t, index_KBx = await KickbackMotion(attempt, 65, 2.9, 3.0, servo, init_euler_xyz, pr, t)

        if success == False:
             attempt = False
        
        data_LRx, t, index_LRx = await LeftToRightBarMotion(attempt, data_LR3, nominal_pcws_LR3, recordedData_LR3, test_LR, 1.25, servo, init_euler_xyz, pr, t)
        
        success, data_CATCHx, t, index_CATCHx = await CatchMotion(attempt, 20, 0.8, 1.2, servo, init_euler_xyz, pr, t)'''
   
        success, data_kickback, t, index_KB = await KickbackMotionRL(attempt, 120, -2.5, 3.0, 3.5, servo, init_euler_xyz, pr, t)
#torque_pop, torque_KB, tau_limit_KB        
        
        if success == False:
             attempt = False
        
        recorded_data_RL, t, index_RL = await RightToLeftBarMotion(attempt, data_RL, nominal_pcws_RL, recordedData_RL, test_RL, 1.8, servo, init_euler_xyz, pr, t)

    finally:
        await servo.set_stop()  

        #combine data for 1-3 LR brachiations
        '''
        #One LR brachiation
        recordedData = combine_data(data_KB, recorded_data_LR)
        recordedData = combine_data(recordedData, data_CATCH)
        
        #Second LR brachiation
        recordedData = combine_data(recordedData, data_KB2)
        recordedData = combine_data(recordedData, recorded_data_LR2)
        recordedData = combine_data(recordedData, data_CATCH2)
        #Third LR brachiation
        recordedData = combine_data(recordedData, data_KBx)
        recordedData = combine_data(recordedData, data_LRx)
        recordedData = combine_data(recordedData, data_CATCHx)'''

        #combine data for RL brachiation
        recordedData = combine_data(data_kickback, recorded_data_RL)

        #save data for full motion       
        data=convert_to_NamedTuple(recordedData)
        
        #data=convert_to_NamedTuple(data_KB) #saving just one motion for later analysis 

        fullLengthIndex = data.n

        store_data={
            field:data[i][:fullLengthIndex] for i, field in enumerate(data._fields)
            if isinstance(data[i], np.ndarray)
        }        
        directory = plot_custom_data(store_data, 'RL_pd_1X_Sept2')       
        # Save Trajectory to a csv file to be sent to the motor.
        save_data(store_data, directory+'/measured.csv')  
            

if __name__ == '__main__':
    
    #ZL motion
    test = 'pd' #'tvlqr'#'ff_replay'#
    folder_name = 'data/trajectories'
    file_name = 'RL_PD.csv'
    tau_limit = 2.5
    bus_number = 4
    motor_id = 7    
    
    csv_data= read_data(folder=folder_name,
                        file=file_name,
                        up_directory=1)                    
    
    #zero the angle values
    zero_offset(bus_number, motor_id)

         #ALL LR motions
    test_LR = 'pd'#'ff_replay'#'pd' #
    test_RL = 'pd' #'tvlqr'#'ff_replay'#
    file_name_LR = 'LR_v2_Sept1_1000Hz.csv' #'traj_RLearn_checkpoint_550000.csv' #'LR_v2_2_1000Hz.csv' # 'LR_15_1000Hz.csv' #'LR_0100.csv' # 'traj_RLearn_checkpoint_100000.csv' #
    file_name_RL = 'RL_PD.csv' #'RL_4_1000Hz.csv' # 'LR_15_1000Hz.csv' #
    csv_data_LR = read_data(folder=folder_name,
                        file=file_name_LR,
                        up_directory=1)
    csv_data_LR2 = read_data(folder=folder_name,
                    file=file_name_LR,
                    up_directory=1)
    csv_data_LR3 = read_data(folder=folder_name,
                file=file_name_LR,
                up_directory=1) 
    csv_data_RL = read_data(folder=folder_name,
                    file=file_name_RL,
                    up_directory=1)                                      

    #number of brachiations desired
   
    if input('continue with brachiation? if so press y:') == 'y':
        asyncio.run(main(csv_data,csv_data_LR,csv_data_LR2,csv_data_RL,test,test_LR,test_RL))

