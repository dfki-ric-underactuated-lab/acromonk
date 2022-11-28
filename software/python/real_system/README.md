

# Trajectory Stabilization with AcroMonk
The following instruction assums that the scripts will be executed on the Raspberry Pi. The required packages that needs to be installed on the Raspberry Pi is provided in the [requirement.txt](requirements.txt).

A Makefile is provided in the following folder for the automatic execution of the robust forward brachiation using TVLQR and PD controller:
`software/python/real_system/Makefile`

The following parameters can be used for arbitrary behavior:

```
TEST          : stabilizing controller (pd, tvlqr), default=pd
FILE          : swing trajectory (BF trajectory in data/trajectories/closed_loop folder), default=BF_tvlqr.csv
MOTOR_ID      : mjbots servo id, default=7
BUS_NUMBER    : bus number on pi3hat, default=4
FILE_RECOVERY : recovery trajectory (ZF trajectory in data/trajectories/closed_loop folder) ,default=ZF_tvlqr.csv
INIT_CONF     : initial configuration of the AcroMonk (1, 2), default=1: hanging imu arm
NBRACH        : number of brachiations, default=1
```
for example:

```
make NBRACH=3  TEST=tvlqr
```

and the plots and csv file are saved in the `results/real_system` folder. 


