This document provides a guide and recommended links for sensor reading of the AcroMonk. 

<div align="center">

# Sensor reading
</div>

AcroMonk use a [pi3hat from mjbots](https://mjbots.com/products/mjbots-pi3hat-r4-4b) for motor communation and IMU sensor reading. 
- [Documentation](https://github.com/mjbots/moteus)
- [Setup guide](https://www.youtube.com/watch?v=aJKB5TGMtuI&t=385s)
## Actuator

A [qdd100 beta 3 servo](https://mjbots.com/collections/servos-and-controllers/products/qdd100-beta-3) is used as an actuator for the robot. 
Here are the useful links for documentation and also setting up the mjbots servo motor:
- [Documentation](https://github.com/mjbots/pi3hat)
- [Setup guide](https://www.youtube.com/watch?v=HHCBohdrCH8)

## IMU sensor reading
In order to read IMU data, one can use pi3hat_tools library provided by mjbots. 
- [pi3hat_tools](https://github.com/mjbots/pi3hat/releases/download/0.1-20210609/20210609-pi3hat_tools-d1e8aa529fe9aa62e6c0df19f10b83bd0e743273.tar.bz2)
- [IMU reading](https://github.com/mjbots/pi3hat/blob/master/lib/python/examples/imu_example.py)

