# Mechanics
The main idea for the design of the AcroMonk is to have a portable and robust agile robot with a minimum effort for the modification. 
A modular design is proposed that uses 3D printing technology for ease of reproducibility. 
Overall, the structure consists of six unique 3D-printed parts highlighted with different colors that are connected by screw-nut fasteners for easy assembly, with compartments for electronics, a battery, counterweights, and cable guides. 

<div align="center">
<img width="400" src="../images/exploded-view.png" />
<img width="400" src="../images/acrm-3d.jpg" />
</div>


**Assembly video needs to be added as gif**

## Hook design
The innovative hook design of the AcroMonk makes him the first brachiator that can brachiate continuously with the passive gripper, depicted in the following figure. This unique design offers a large region of attraction, shown in orange color, for grasping a target bar and a well defined pivot point for the swing maneuvers. 
<div align="center">
<img width="500" src="../images/2d-arm-acrm-coordinate_v2.png" />
</div>

# Electronics
The schematic of the electrical diagram with the components are shown in the following figure.

<div align="center">
<img width="800" src="../images/wiring-diagram.png" />
</div>


## Components 
### Battery
All electronics of the AcroMonk are powered by a LiPo battery with the following technical data:
- Capacity: $`6\text{S}\ 1200 \text{ mAh}`$ 
- Voltage: $`22.2\text{V}`$
- Continuous Discharge: max. $`30\text{C } (36\text{A})`$
- Burst Discharge:max. $`60\text{C } (72\text{A})`$
- Power: $`26.64 \text{ WH}`$

### Raspberry Pi 
A Raspberry Pi 4 Model B is used for the onboard computer, providing wireless communication capability, a setup guide is explained [here](wireless-communication.md). 

### pi3hat
[Mjbots pi3hat](https://mjbots.com/products/mjbots-pi3hat-r4-4b) is a daughterboard for the Raspberry Pi, and sits on top of it by connecting through the $`40`$ pin GPIO interface. The $`\text{XT}30\text{-M}`$ power connector on pi3hat enables the powering of the Raspberry Pi, and also provides CAN interfaces for communication with the actuator ([mjbots qdd 100 servo](https://mjbots.com/collections/servos-and-controllers/products/qdd100-beta-3)) and IMU sensor reading. A setup guide for the motor configuration and IMU sensor reading of AcroMonk is provided in the following: 
- [Mjbots QDD 100 servo](motor-configuration.md)
- [IMU sensor reading](imu-reading.md)

### Emergency Stop
To have the authority to disable the motor in case of the undesired behavior, a remote emergency stop is designed. A relay, diode, and a $`30 \text{A}`$ electronic switch are employed to disconnect the power. The triggering command is provided by a joystick that connects the electronic switch with an onboard receiver, which is powered by a buck converter.
A series combination of Varistor(voltage-dependant-resistor) and a resistor component is used to protect the buck conrverter from high-frequency voltage fluctuations.

# Mechatronics Integration
Here is the final integartion of the AcroMonk.
<div align="center">
<img width="300" src="../images/integration-front.JPG" />
<img width="277" src="../images/integration-back.JPG" />
<img width="328" src="../images/integration-es.JPG" />
</div>
