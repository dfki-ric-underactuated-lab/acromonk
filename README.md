# AcroMonk: A Minimalist Underactuated Brachiating Robot
<div align="center">
<img width="605" src="images/poster_new.png" />
</div>


<div align="center">
<img width="605" src="images/5x-tvlqr.gif" />
</div>

<div align="center">
<img width="300" src="images/zb.gif" >
<img width="300" src="images/bf.gif" >
</div>
<div align="center">
<img width="300" src="images/zf.gif" >
<img width="300" src="images/fb.gif" >
</div>

## Description
This project offers an open-source and low-cost kit to test control algorithms 
for underactuated robots. 
it implements a **brachiation robot** platform called **AcroMonk** which 
can brachiate robustly over a horizontally laid ladder bar. AcroMonk is 
the simplest possible underactuated brachiator built using one quasi-direct 
drive actuator (QDD) and passive grippers. This is the first brachiator with 
unactuated grippers that can perform more than two brachiation maneuvers. 
This project offers different control methods for trajectory stabilization 
which can be studied using the kit. Additionally, it provides a list of 
components, discusses best practices for implementation, and presents results 
from experiments with the simulator and the real system. This repository 
describes the hardware (CAD, Bill Of Materials ([BOM](hardware/bills-of-materials.md)) etc.) required to build 
the physical system and provides the software (URDF models, simulation and 
controllers) to control it.


## Documentation

The [hardware setup](hardware/testbench-description.md) and [sensor reading](hardware/sensor-reading.md) are described in their respective readme files. The dynamics of the AcroMonk are explained [here](docs/acrm-equations.md).

* [Hardware & Testbench Description](hardware/testbench-description.md)
* [Sensor reading](hardware/sensor-reading.md)
* [Bill Of Materials (BOM)](hardware/bills-of-materials.md)

## Authors #

* [Shivesh Kumar](https://robotik.dfki-bremen.de/en/about-us/staff/shku02.html) (Project Supervisor)
* [Mahdi Javadi](https://robotik.dfki-bremen.de/en/about-us/staff/maja04/) (Hardware and Software Maintainer)
* [Daniel Harnack](https://robotik.dfki-bremen.de/en/about-us/staff/daha03.html)
* [Shubham Vyas](https://robotik.dfki-bremen.de/en/about-us/staff/shvy01/)
* [Daniel Pizzutilo](https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/dapi01.html) (Mechanical Design)
* Paula Stocco

Feel free to contact us if you have questions about the test bench. Enjoy!

## Contributing

1. Fork it (<https://github.com/yourname/yourproject/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request


## Safety Notes #

When working with a real system be careful and mind the following safety measures:

* Brushless motors can be very powerful, moving with tremendous force and speed. Always limit the range of motion, power, force and speed using configurable parameters, current limited supplies, and mechanical design.

* Stay away from the plane in which pendulum is swinging. It is recommended to have a safety net surrounding the pendulum in case the pendulum flies away.

* Make sure you have access to emergency stop while doing experiments. Be extra careful while operating in pure torque control loop.

## Acknowledgements #
This work has been performed in the VeryHuman project funded by the German Aerospace Center (DLR) with federal funds (Grant Number: FKZ 01IW20004) from the Federal Ministry of Education and Research (BMBF) and is additionally supported with project funds from the federal state of Bremen for setting up the Underactuated Robotics Lab (Grant Number: 201-001-10-3/2021-3-2).

<img width="500" src="docs/reference/source/figures/Logo_Underactuated_Lab.gif" />


## License

This work has been released under the BSD 3-Clause License. Details and terms of use are specified in the LICENSE file within this repository. Note that we do not publish third-party software, hence software packages from other developers are released under their very own terms and conditions, e.g. Stable baselines (MIT License) and Tensorflow (Apache License v2.0). If you install third-party software packages along with this repo ensure  that you follow each individual license agreement.   

## Citation

1. Wiebe et al., (2022). Torque-limited simple pendulum: A toolkit for getting familiar with control algorithms in underactuated robotics. Journal of Open Source Software, 7(74), 3884, https://doi.org/10.21105/joss.03884
```
@article{Wiebe2022,
  doi = {10.21105/joss.03884},
  url = {https://doi.org/10.21105/joss.03884},
  year = {2022},
  publisher = {The Open Journal},
  volume = {7},
  number = {74},
  pages = {3884},
  author = {Felix Wiebe and Jonathan Babel and Shivesh Kumar and Shubham Vyas and Daniel Harnack and Melya Boukheddimi and Mihaela Popescu and Frank Kirchner},
  title = {Torque-limited simple pendulum: A toolkit for getting familiar with control algorithms in underactuated robotics},
  journal = {Journal of Open Source Software}
}
```
