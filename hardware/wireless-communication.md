This is a guide provides information on how to setup the Raspberry pi 4 and access to the AcroMonk remotely. 

## Setup Raspberry Pi
Here is [a useful guide](https://projects.raspberrypi.org/en/projects/raspberry-pi-setting-up) how to setup a Raspberry Pi.

## Remote Access
In the follwing, the steps that needs to be taken to access the onboard Raspberry Pi are listed.
### Wireless connection
In order to access the robot remotely, one of the following strategies can be followed:
- Setup the Raspberry pi as a wireless access point ([link](https://raspberrypi-guide.github.io/networking/create-wireless-access-point))
- Setup a router and connect the Raspberry Pi and server to the same router
### Obtain the IP address
To get the IP adderss you need to open a terminal in your Raspberry Pi OS use the following command:
```
hostname -I
```
### Enable SSH
One way to enable the SSH is to use the following command in **terminal**:
```
sudo raspi-config
```
select the `interfacing Options`. Navigate to and select `SSH`. Choose `Yes`. Select `OK`. Choose `Finish`.

### Access via SSH

Using the IP address that is obtained in the previous step, one can connect to Raspberry pi by typing the following command in terminal:

```
ssh pi@<IP>
```