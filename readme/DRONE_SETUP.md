# Drone setup

Step-by-step instructions to setup the drone

## Flight controller configuration

- Connect the flight controller to you PC by USB
- Launch [inav-configurator 2.3.2](https://github.com/iNavFlight/inav-configurator/releases/tag/2.3.2)
- Open `CLI` and type:
```terminal
dfu
```
- go to the `firmware flasher` tab
- make sure `full chip erase` is selected
- click `load firmware`
- navigate to the `.hex` file
- select `flash`
- go to the inav configurator `CLI`, select `load from file`
- navigate to the `.txt` file
- type:
```terminal
save
```
- with a battery connected to the drone, go to inav configurator `setup` tab and check that everything is green
- disconnect the battery

## raspberry pi configuration

- install [raspbian](https://www.raspberrypi.org/software/operating-systems/) on the SD card (you can use Raspberry Pi OS Lite)
- configure [SSH access](https://phoenixnap.com/kb/enable-ssh-raspberry-pi):
  - create an empty file named `ssh` in the root of the `boot` partition of the SD card

- configure wpa-supplicant so the drone connects to your router:
  - open `etc/wpa_supplicant/wpa_supplicant.conf` in the `rootfs` partition
  - adapt the following and add it to the end of the `wpa_supplicant.conf` file:
    ```terminal
    network={
            ssid="yourNetworkSSID"
            psk="yourNetworkPassword"                                       
            scan_ssid=1
    }
    ```

- change the name of the drone:
  - open the `etc/hostname` file
  - write your drone name instead of the default `raspberrypi`
  - open the `etc/hosts` file
  - change the last line (by default `raspberrypi`) with your drone name

- plug the SD card into the raspberry pi and connect a battery to start the drone
- ssh the drone from your computer, the default password in raspbian should be `raspberry`:
```terminal
ssh pi@myDroneName.local
```
- install `pip3`:
```terminal
sudo apt-get install python3-pip
```
