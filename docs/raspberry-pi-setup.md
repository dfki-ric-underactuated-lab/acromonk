This is a guide on how to setup the Raspberry pi 4 for the acroMonk. 

### Setup Raspberry Pi
Download and install _Raspberry Pi OS on the Raspberry Pi([click on me!](https://www.raspberrypi.com/software/))

### Raspberry Pi ports
After the installation of the OS on a micro SD you need to insert the the micro SD in the slot. 
**Image needs to be added**
You need to connect the following components to the corresponding ports:
**Image needs to be added**
1- power -> USB type-c
2- mouse -> USB type 2
3- keyboard -> USB type 2
4- External monitor -> micro HDMI port
- Note: if you just have 1 external monitor, you might use the **HDMI0** port, which is near the power port 


5- (optional): LAN cable -> Ethernet port


### Remote Access
In order to get access to your onboard Raspberry Pi, you might need:
- IP address of the Raspberry Pi
- Enable the SSH/VNC

#### Get IP address
To get the IP adderss you need to open a terminal in your Raspberry Pi OS use the following command:
```
hostname -I
```
Then the ip adress will be showed up! The ip address for the current Rasperry Pi mounted on the AcroMonk is :
```
10.250.6.39
```

#### Enable SSH/VNC
To enable the SSH/VNC, you can either use the **terminal**:
```
sudo raspi-config
```
select the `interfacing Options`. Navigate to and select `SSH`. Choose `Yes`. Select `OK`. Choose `Finish`.

Alternatively, you can use the **GUI** option. 
Click on the `Raspberry Pi-icon on top left corner > Prefernces > Raspberry Pi Configuration`.
Go to the interface tab and enable `SSH/VNC` and click on the `OK`. 

Note: In order to check the connection of the Raspberry Pi to the internet you might use the following command:
```
ping raspberyypi.local
```

### Connect to Raspberry Pi via SSH

In order to connect to the Raspberry Pi from another PC/laptop, you need to use the following command:
```
ssh pi@<IP>
```
As described above, the IP for the current Raspberry Pi mounted on the AcroMonk is `10.250.6.39`. So the following command will connect you to the AcroMonk onboard Raspberry Pi:
```
ssh pi@10.250.6.39
```
It askes for the connection, and you need type `yes`.
Then you might need a password which is `utbed`, (as the password for the VeryHuman PC in Lab). Finally you can see `pi@raspberrypi:` infront of the directory path, which verifies your successful connection to the Raspberry Pi!


### Setup a D-Link router
In order to setup a `ASUS` router, you need to follow the steps:
- Plug in the power and check the lights
- Plugin one end of a LAN cable to the `LAN port` on the back of the router and the other end to any Ethernet port of your Laptop/PC. This will be needed for configuration of the router.
- Open a browser and type `192.168.1.1`.
- Navigate trough `wireless` tab. You might need a user credential: 
```
username  = admin
password = admin
``` 
Setup two channels `2.4 GHz` and `5 GHz` as shown below in the picture:

**INSERT THE PICTURE**

- For the password and name of the SSID, navigate through the `Wireless Security` tab. You might set password for both channels. 
Current configuration is shown in th picture below. 
We have 2 channels and we have set the same password for both:
```
SSSID: acrm_2.4
password:underactuatedtestbench
SSSID: acrm_5
password:underactuatedtestbench
```

### Connecting laptop and Raspberry Pi through WiFi
In order to access the Raspberry Pi through wifi from the laptop computer, you need to connect both to the same router. 
`Hint`: it is not important that both connect to the same channel

#### Setting up a static IP

Every time that Raspberry Pi starts up, it sets a new IP address. It would be handy to set a static IP for that. 
For this purpose you need to follow the steps below:
- Right click on the `wifi icon` on top right corner and select `Wireless & Wired Network Settings`.
- Click on the 2nd box in the same row of the `Configure:` and select `wlan 0`
- Set the `IPv4 Address:` to any arbitrary valid IP address
- Set the `Router` to the `192.168.1.1`. This is the same gateway that you had access to configuration of the router.
- Set the DNS Servers to `8.8.8.8`
A summary of the steps can be reviewed in the following picture

`Insert the picture`

Then you can access the Raspberry Pi through this static IP by typing the following command in the termianl:
```
ssh pi@192.168.1.18
```
and then insert the password as `utbed`.

#### View the Raspberry Pi Desktop with VNC Viewer

In order to access the graphical GUI of the Raspberry Pi, the easiest way is to use the `vnc viewer`.
Follow the steps:
- Navigate trough [this link](https://www.realvnc.com/en/connect/download/viewer/). Select your operating system and download the installation file.
- In Ubuntu, you can navigate to the downloaded folder and right click on the `.deb` file and select `open and software install` and complete the installation. This can alternatively be done by ```sudo dpkg -i package_file.deb```
- After installation, you can access vnc viewer by typing `vncviewr` in your terminal(you might need to restart the termainal).
- Open the VNC Viewr and in the search bar serach for the static IP that you have set before: `192.168.1.18`.
- Enter the password for the `pi` user and the desktop will shows up!

### Other Functionalities
#### Sending a file from Laptop to Raspberry Pi via WiFi
```
scp path/to/file pi@192.168.1.18:/path/to/copy/in/rpi
```
Then it will be prompted to enter the password of the Raspberry Pi, which is `utbed`.

[Main Menu](https://git.hb.dfki.de/underactuated-robotics/acroMonk)