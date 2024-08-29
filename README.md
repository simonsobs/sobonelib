# sobonelib
This repository contains code which runs on Beaglebone Black microcontrollers. This code is ment to interact with OCS agents but does not fit within the standard agent structure. Instead these pieces of code run as independent processes on the Beaglebone microcontrollers and interact with agents through UDP data packets.

## Beaglebone Black Microcontrollers
[Beaglebone Black's](https://beagleboard.org/black) are off the shelf microcontrollers with ~80 configurable pins (GPIO, PWM, PRU, etc) and an onboard linux kernal. Each Beaglebone has two so called "Programmable Real time Units (PRUs)" which are able to independently run compiled code on 200 MHz processors. These PRUs have access to shared memory addresses and can query/change the state of specific PRU pins. The 200 MHz clock speed is a significant improvement over similar microcontrollers (16 MHz for Arduino Microcontrollers) and improves accuracy when measuring the state of a rapidly changing signal. Unfortunately, programming with the PRU's is not well documented and requires additional steps beyond what a typical user might understand.

## hwp_encoder
Beaglebone code for the hwp_encoder agent. This code continually records rising/falling edges from two encoder signals (one primary one quadrature) and one IRIG-B timing signal. Encoder collection runs on PRU1 while IRIG-B collection runs on PRU0. Data is stored at shared memory addresses so it can be put into packets by the main processor.

### Setup
#### Network Setup
- Connect to the beaglebone with the provided USB cable using the default credentials `ssh debian@192.168.7.2`, password: `temppwd`
- (Optional) Create a new user using `sudo adduser [username]` and `sudo usermod -aG sudo [username]`
- Make the IP of the beaglebone static, so that the board can be SSH-ed into over ethernet, by opening the file `/etc/network/interfaces` and make the following changes (Commented lines have been omitted):

Before
```
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet dhcp
```
After
```
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet static
        address [user defined]
        netmask [user defined]
        network [user defined]
        gateway [user defined]

iface usb0 inet static
        address 192.168.7.2
        netmask 255.255.255.252
        network 192.168.7.0
        gateway 192.168.7.1
```
- Reboot the beaglebone with `sudo reboot`. Connect the beaglebone to your network switch and varify that the beaglebone is accessible at the entered network configuration
  
#### PRU Config
- In order to use the beaglebone PRUs for DAQ, uncomment the following lines in `/boot/uEnv.txt`
  - `disable_uboot_overlay_video=1`
  - `disable_uboot_overlay_audio=1`
  - `uboot_overlay_pru=/lib/firmware/AM335X-PRU-UIO-00A0.dtbo`
 - Once the `/boot/uEnv.txt` file has been edited, reboot the beaglebone with `sudo reboot` to apply the changes

#### External Package Installation
- Clone the [am335x_pru_package](https://github.com/beagleboard/am335x_pru_package) repository onto the beaglebone
- Navigate into the top level directory `cd am335x_pru_package` and then run `make` followed by `make install`
- Navigate to `am335x_pru_package/pru_sw/app_loader/lib/` and copy the generated binaries `libprussdrv.a`, `libprussdrv.so`, `libprussdrvd.a`, and `libprussdrvd.so` into `/usr/lib/`

#### Building sobonelib/hwp_encoder
- Navigate into the hwp_encoder subdirectory `cd sobonelib/hwp_encoder`
- Edit the `config.txt` file so that the `COM_PORT` and `HOST_IP` match your agent setup
  - By default `COM_PORT` should be 8080 for encoder1 and 8081 for encoder2
  - `HOST_IP` is the IP address of the node running the encoder OCS agent
 - With the configuration file set, run the command `make`.
   - This will both compile the code and instantiate the systemctl services

#### Checking Status
- To check the status of the encoder process, use `systemctl status hwp-encoder-pru.service`
- To stop/start the encoder process, use `sudo systemctl stop hwp-encoder-pru.service`/`sudo systemctl start hwp-encoder-pru.service`
- To change the configuration parameters, simply update `config.txt` and run `make` again
- To get more detailed logs from the process, use `journalctl -u hwp-encoder-pru.service -n 100`

## hwp_gripper
Beaglebone code for the hwp_gripper agent. This code is split into two independent processes. The first process runs on the PRU's and periodically records the state of six encoder signals (two for each actuator) and six limit switch signals (two for each actuator; one for the warm grip location and one for the cold grip location). Encoder collection runs on PRU1 while limit switch collection runs on PRU0. Data is stored at shared memory addresses so it can be put into packets by the main processor. The second process is a simple python script, which acts a server to process incomming control commands. This script accepts a socket connection from the agent and converts commands received into state changes of the Beaglebone's pins.

### Setup
The hwp_gripper setup is largely similar to the hwp_encoder setup with a couple key differences. As a background it is assumed that you have read through and understood the hwp_encoder setup section.

#### (New) Installing OS onto microSD
- Typically the beaglebone OS is stored on an embedded MultiMediaCard (eMMC). For the hwp_gripper however, serveral I/O pins typically reserved for the eMMC are reappropriated to different tasks. This means that the OS needs to run from an external >=32 GB microSD card instead
- On a seperate computer, download the beaglebone OS from [here](https://www.beagleboard.org/distros). Filter for "BeableBone Black" and select a non-flasher debian image that runs on an SD card. Previous beablebones have used `AM3358 Debian 10.3 2020-04-06 4GB SD IoT`
- Download [Etcher](https://etcher.balena.io/) onto the same computer, and use it to flash the OS onto the microSD card
- Insert the microSD into the beaglebone's microSD slot and supply power
- Uncomment the following lines in `/boot/uEnv.txt`
  - `disable_uboot_overlay_emmc=1`
  - `disable_uboot_overlay_wireless=1`
  - `disable_uboot_overlay_adc=1`
 - Reboot the beaglebone with `sudo reboot`. The beaglebone OS should now be running from the microSD card

#### Network Setup
- Exactly the same as hwp_encoder's Network Setup

#### PRU Config
- Exactly the same as hwp_encoder's PRU Config

#### External Package Installation
- Exactly the same as hwp_encoder's External Package Installation

#### Building sobonelibe/hwp_gripper
- Navigate into the hwp_gripper subdirectory `cd sobonelib/hwp_gripper`
- Edit the `config.txt` file so that the `PRU_PORT` and `CONTROL_PORT` match your agent setup
  - By default `PRU_PORT` should be 8040 and `CONTROL_PORT` should be 8041
- With the configuration file set, run the command `make`.
   - This will both compile the code and instantiate the systemctl services

#### Checking Status
- To check the status of the gripper processes, use `systemctl status hwp-gripper-pru.service` and `systemctl status hwp-gripper-control.service`
- To stop/start the gripper processes, use `sudo systemctl stop hwp-gripper-pru.service`/`sudo systemctl start hwp-gripper-pru.service` and `sudo systemctl stop hwp-gripper-control.service`/`sudo systemctl start hwp-gripper-control.service`
- To change the configuration parameters, simply update `config.txt` and run `make` again
- To get more detailed logs from the processes, use `journalctl -u hwp-gripper-pru.service -n 100` and `journalctl -u hwp-gripper-control.service -n 100`
