
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

---

## 2. Topside

#### Calibration:
```sh
# collect mag data

# do calibration
roslaunch calibrate_imu calibrator.launch
```
---

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


-----

## 4. Camera:
- lab light condition + led 1900: expsoure can go to 4900+
  
- lab light condition, normal expsoure is 15000+
  
- **set max exposure as ligh as possible, when light enough, exposure time will adjust itself.**
  
- **in low light condition, larger max_exposure will help. But in some level, it will not help anymore. reduce it to get good image, otherwise may get motion blur**
  
- **when adjust max_ exposure, remember apply this max_epxosure to both cam(primary and secondary)**