
# NOTES

## 1. Jetson

#### Check data
```sh
# check disk size
df -H
# check folder size
du -hs /path/to/directory
# check all the files size by order
du -h | sort -h
# transfer file from Jetson to portable external usb drive
rsync --info=progress source dest
# find usb drive
sudo fdisk -l
# reject usb drive
sudo eject /dev/sda
```

#### print unix epoch
```sh
watch -n -0.1 date +%s
```

### Setup Shared Desktop: [reference](https://www.digitalocean.com/community/tutorials/how-to-install-and-configure-vnc-on-ubuntu-18-04)

**Environment setup:** 
```sh 
# update
$ sudo apt-get update
# install the Xfce desktop environment on your server
$ sudo apt install xfce4 xfce4-goodies 
# install the TightVNC server
$ sudo apt install tightvncserver
# create password and initial configuration
vncserver
```

**Configuring the VNC Server:**
```sh
# kill existing server before configure VNC server
$ vncserver -kill :1
# backup original file
$ mv ~/.vnc/xstartup ~/.vnc/xstartup.bak
# make new file and copy following 
$ vim ~/.vnc/xstartup
```
> #!/bin/bash  
> xrdb $HOME/.Xresources
> startxfce4 &  

```sh
# make it executable
$ sudo chmod +x ~/.vnc/xstartup
# restart it 
$ vncserver
```

**Connecting the VNC Desktop:**
- Securely connection: `ssh -L 5901:127.0.0.1:5901 -C -N -l soslab 192.168.2.3`
- use VNC client to log in server: `localhost:5901` with initialized passward

**Running VNC as a System Service:**
- create system servce: `sudo vim /etc/systemd/system/vncserver@.service`  
- copy following into servce:
    >[Unit]  
    >Description=Start TightVNC server at startup  
    >After=syslog.target network.target  
    >
    >[Service]  
    >Type=forking 
    >User=soslab 
    >Group=soslab 
    >WorkingDirectory=/home/soslab 
    >
    >PIDFile=/home/soslab/.vnc/%H:%i.pid 
    >ExecStartPre=-/usr/bin/vncserver -kill :%i > /dev/null 2>&1 
    >ExecStart=/usr/bin/vncserver -depth 24 -geometry 1280x800 :%i 
    >ExecStop=/usr/bin/vncserver -kill :%i 
    > 
    >[Install] 
    >WantedBy=multi-user.target 
- test service:
    ```sh
    # reolad systemctl configuation
    $ sudo systemctl daemon-reload
    # enable this service at boot
    $ sudo systemctl enable vncserver@1.service
    # start first one, you can start a lot
    $ sudo systemctl start vncserver@1
    # check
    $ sudo systemctl status vnserver@1
    ```
- start ssh in local: `ssh -L 5901:127.0.0.1:5901 -C -N -l soslab 192.168.2.3`
- start VNC viewer using `localhost:5901`and created password.

----------

## 2. Topside

#### Calibration:
```sh
# collect mag data

# do calibration
roslaunch calibrate_imu calibrator.launch
```
---

#### UDEV rules
```sh
# check device: Bus 001 Device 011: ID 1546:01a7 U-Blox AG
lsusb 
# add udev rules
sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ACTION==\"add\", ATTRS{idVendor}==\"1546\", ATTRS{idProduct}==\"01a7\", SYMLINK+=\"vfan_gps\"" > /etc/udev/rules.d/99-usb-gps.rules'
sudo reboot
```

#### Kill process
```sh
ps aux | grep -i name
sudo kill -9 pid
```

## 3. Pi

**Connection**:
```sh
ssh pi@192.168.2.2 # password: companion
```

**Service meaning:**
- mavproxy connects QGC to the Pixhawk
- mavlink2rest provides a mavlink terminal in the webui (can comment out, always possible to restore later if you for some reason need it for something)
- bridgemanager enables ping360 connection
- wldriver is for water linked underwater GPS (can comment)
- nmearx is for NMEA GPS (can comment if not using)
- file-manager allows you to get files off the companion computer (can comment if not using, also possible to do directly with scp instead)
- audio is saved as part of video recordings, although QGC doesn’t play it live at the moment
- commrouter is for forwarding connections, at the /routing page of the web ui (not essential if you’re not using any such connections)
- webterminal is the terminal you’re using (probably good to keep, but not essential if you know how to use ssh, user:`pi`, pass: `companion` )
- webui is the companion web interface
- video is video stream

**Pi monitor system:** http://192.168.2.2:2770/system

**Check pi:**

```shell
$ ssh pi@192.168.2.2 # password: companion
$ vcgencmd measure_temp # monitor temperature
```

**MAVproxy:**

```shell
--out udpout:localhost:9002 #mavlink2rest, removed; 
--out udpin:0.0.0.0:14660   #ping360 heading integration
# referece: 
# https://discuss.bluerobotics.com/t/lost-manual-control/7920/3
```

---

## 3. Arduino

**Compile**:
```sh
# check device
ls -ln /dev/arduino_zero
# upload
cd ~/Develop/dev/arduino
arduino --upload synchronizer_arduino/synchronizer_arduino.ino --port /dev/ttyACM1
# verify 
arduino --verify synchronizer_arduino/synchronizer_arduino.ino
```

**Rebuild ros_lib**:

```shell
 cd ~/path/Arduino/libraries/
 rm -rf ros_lib
 rosrun rosserial_arduino make_libraries.py .
 cp ~/path/ArduinoHardware.h ../ros_lib
```

**ROS-serial issue:**
> [INFO] [1647037283.090613]: wrong checksum for msg length, length 32  
> [INFO] [1647037283.098949]: chk is 125  
> [ERROR] [1647037283.109608]: Mismatched protocol version in packet ('\xff'): lost sync or rosserial_python is from different ros release than the rosserial client  
> [INFO] [1647037283.115555]: Protocol version of client is Rev 0 (rosserial 0.4 and earlier), expected Rev 1 (rosserial 0.5+)  

[soluation]((https://answers.ros.org/question/199490/ros-arduino-message-type-restrictions/))

-----

## 4. Camera:
- lab light condition + led 1900: expsoure can go to 4900+
  
- lab light condition, normal expsoure is 15000+
  
- **set max exposure as ligh as possible, when light enough, exposure time will adjust itself.**
  
- **in low light condition, larger max_exposure will help. But in some level, it will not help anymore. reduce it to get good image, otherwise may get motion blur**
  
- **when adjust max_ exposure, remember apply this max_epxosure to both cam(primary and secondary)**


## Time synchronization:
example: Arduino time: 19.500, Jetson clock: 19.400(synchronized with Arduino, but -100ms offset), image arrived at jetson at 19.420, that's why still eariler then Arduino tigger time.

**Cam:** 
- time start:(cheony timeoffset between Arduino~Jetson:100ms)
    - sync time:
        - secs: 1647036462
        - nsecs: 561935000
    - io_time: 
        - secs: 1647036462
        - nsecs: 417867599

- time end: 15 min (cheony timeoffset between Arduino~Jetson:90ms)
    - sync time:
        - secs: 1647037112
        - nsecs: 961966000
    - io_time: 
        - secs: 1647037112
        - nsecs: 826078502

- time end: 30 min(cheony timeoffset between Arduino~Jetson:80ms)
    - sync time:
        - secs: 1647038331
        - nsecs: 561907000
    - io_time: 
        - secs: 1647038331
        - nsecs: 457900244

- time end: 45 min (cheony timeoffset between Arduino~Jetson:50ms)
    - sync time:
        - secs: 1647039353
        - nsecs: 461950000
    - IO time: 
        - secs: 1647039353
        - nsecs: 454006663

- time end: 60 min (cheony timeoffset between Arduino~Jetson:60ms)
    - sync time:
        - secs: 1647040159
        - nsecs: 361988000
    - io_time: 
        - secs: 1647040159
        - nsecs: 248997090


**AHRS:**
- time start:(cheony timeoffset between Arduino~Jetson:100ms)
    - sync time:
        - secs: 1647036394
        - nsecs: 281000000
    - IO time:
        - secs: 1647036394
        - nsecs: 190887936

- time end: 15 min (cheony timeoffset between Arduino~Jetson:80ms)
    - sync time:
        - secs: 1647037056
        - nsecs: 451000000
    - IO time:
        - secs: 1647037056
        - nsecs: 369074432

- time end: 30 min (cheony timeoffset between Arduino~Jetson:80ms)
    - sync time:
        - secs: 1647038541
        - nsecs: 152000000
    - IO time:
        - secs: 1647038541
        - nsecs: 132259840

- time end: 45 min (cheony timeoffset between Arduino~Jetson:50ms)
    - sync time:
        - secs: 1647039397
        - nsecs: 405000000
    - IO time:
        - secs: 1647039397
        - nsecs: 450022144

- time end: 60 min (cheony timeoffset between Arduino~Jetson:60ms)
    - sync time:
        - secs: 1647040193
        - nsecs: 198000000
    - IO time:
        - secs: 1647040193
        - nsecs: 143339008

**DVL:**
- time sync:
    secs: 1647035424
    nsecs: 616051912
- time IO: 
    secs: 1647035424
    nsecs: 541389062

