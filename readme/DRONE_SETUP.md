# Drone setup

Follow these step-by-step instructions to properly setup the CogniFly drone from zero.

## Flight controller configuration
- download both the [cognifly framework](https://github.com/thecognifly/cognifly-python/releases/download/v0.0.4/cognifly_framework.hex) and the [cognifly configuration](https://github.com/thecognifly/cognifly-python/releases/download/v0.0.4/cognifly_configuration.txt) files
- connect the flight controller to your PC through USB
- launch [inav-configurator 2.3.2](https://github.com/iNavFlight/inav-configurator/releases/tag/2.3.2) (on linux, do it with `sudo`)
- `connect` to the flight controller
- open the `CLI` tab and enter the following command:
```terminal
dfu
```
This will disconnect the flight controller and restart it in DFU mode.
- open the `firmware flasher` tab
- make sure `full chip erase` is selected
- click `load firmware [local]`
- select the cognifly framework file previously downloaded (`.hex` file)
- click `flash firmware` and wait for completion
- `connect` to the flight controller, select keep defaults, let it reboot, `connect` again
- open the `CLI` tab and select `load from file`
- navigate to the cognifly configuration file previously downloaded (`.txt` file)
- click `execute` and wait for completion
  - _(NB: check for errors in the logs and ensure the whole .txt has been processed)_
- enter the following command:
```terminal
save
```
This will reboot the flight controller.
- `connect` to the flight controller again
- in the `setup` tab, check that everything is green
- (optional) go to the `calibration` tab and follow the calibration instructions
- go to the `mixer` tab and [follow the inav instructions](https://github.com/iNavFlight/inav/blob/master/docs/Mixer.md) to configure the 4 motors properly
- `disconnect` from the flight controller

## Raspberry Pi configuration

### Raspbian image:

The easiest way of using `cognifly-python` is to flash our readily set up Rasbian image on your SD card.

- download the [Cognifly Raspbian image](https://github.com/thecognifly/cognifly-python/releases/download/v0.2.1/cognifly_image.tar.gz)
- extract the archive
- you should now see the extracted file named `cognifly.img`
- flash `cognifly.img` on your SD card (e.g., using [Win32 Disk Imager](https://win32diskimager.download/) on Windows, or `dd` on Linux)
- in the now flashed SD card, configure wpa-supplicant so the drone connects to your router:
  - edit the `/rootfs/etc/wpa_supplicant/wpa_supplicant.conf` file (on linux, this may require `sudo`) 
    - adapt the following to your WiFi network in the `wpa_supplicant.conf` file:
      ```terminal
      network={
              ssid="yourNetworkSSID"
              psk="yourNetworkPassword"
              scan_ssid=1
      }
      ```
- change the name of the drone:
  - edit the `/rootfs/etc/hostname` file
    - replace the default `K00` by your drone name
  - edit the `/rootfs/etc/hosts` file
    - replace the default `K00` in the last line by your drone name
- plug the SD card into the Raspberry Pi
- plug a charged battery to start the drone
- password for the user **pi**: `maplesyrup`

### Manual installation:

_(Note: if using the Raspbian image, you can ignore this section.)_

In case you wish to install the library on your own Raspbian:

- install [raspbian](https://www.raspberrypi.org/software/operating-systems/) on the SD card (you can use Raspberry Pi OS Lite)

_(Note: You can use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to install raspbian, which provides a GUI to skip most of the following.)_

The SD card should now have two partitions: `boot` and `rootfs`

- configure [SSH access](https://phoenixnap.com/kb/enable-ssh-raspberry-pi):
  - create an empty file named `ssh` at the root of the `/boot/` partition

- configure wpa-supplicant so the drone connects to your router:
  - edit the `/rootfs/etc/wpa_supplicant/wpa_supplicant.conf` file (on linux, this may require `sudo`) 
    - adapt the following and add it to the end of the `wpa_supplicant.conf` file:
      ```terminal
      network={
              ssid="yourNetworkSSID"
              psk="yourNetworkPassword"
              scan_ssid=1
      }
      ```

- change the name of the drone:
  - edit the `/rootfs/etc/hostname` file
    - replace the default `raspberrypi` by your drone name
  - edit the `/rootfs/etc/hosts` file
    - replace the default `raspberrypi` in the last line by your drone name

- enable the pi camera and the UART serial port:
  - edit the `/boot/config.txt` file
    - under the `[all]` section, set the following lines:
      ```terminal
      enable_uart=1 # serial port
      start_x=1 # camera
      gpu_mem=128 # camera
      ```

- disable the console messages in the serial port:
  - edit the `/boot/cmdline.txt` file
    - if you see `console=serial0,115200`, remove this command
- plug the SD card into the Raspberry Pi
- plug a charged battery to start the drone
- wait for the Raspberry PI to boot (the light should stop blinking)
- ssh the drone from your computer (the default password should be `raspberry`):
  ```terminal
  ssh pi@myDroneName.local
  ```
- execute the following on the Raspberry Pi:
  ```terminal
  sudo apt-get update
  sudo apt-get install libatlas-base-dev libopenjp2-7 libtiff5 python3-pip
  ```

You can now install the `cognifly-python` library on the drone.
