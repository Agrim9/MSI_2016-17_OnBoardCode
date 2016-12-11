# MSI_2016-17_OnBoardCode
The repository for the code running over Onboard Microcontroller for Mars Rover Project, IIT-Bombay 2016-17

## Roboclaw Configurations 
One roboclaw is set to address 0x80, other at 0x81, both in packet serial modes. <br/>

* [Roboclaw User Manual](http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf)
* [Roboclaw Datasheet](http://downloads.ionmc.com/docs/roboclaw_datasheet_2x45A.pdf)

## USB settings for roboclaw

Roboclaw1 is the "RightClaw" and should be connected to port 3 of USB hub <br/>
Roboclaw2 is the "LeftClaw" and should be connected to port 4 of USB hub <br/>
The config files can be found in /etc/udev/rules.d/72-micro-devel.rules <br/>
Checking whether the appropriate roboclaw is connected, run this command : <br/>
   `udevadm info -a -n /dev/roboclaw1 | grep 'KERNEL'` to know the tty number <br/>
   `ls -lF /dev | grep roboclaw*` <br/> 
  <br/>
Checking whether it's roboclaw on ttyACM1, run this command : <br/>
   `udevadm info -a -n /dev/ttyACM0 | grep '{product}'`<br/>
<br/>
[More about Udev](http://www.joakimlinde.se/microcontrollers/arduino/avr/udev.php) <br/>

## Roscore doesn't work (1 GB log file error)

Force use roscore with appropriate ROS_IP and ROS_MASTER_URI. No cleanup/etc is required for same, the error is due to some internal SSH kindof error with ROS.

## Configuring RPI for the rover

* *Getting Rpi SSH ready* : Install Raspbian and place empty file named "ssh" (no extensions) in boot directory for sshing to pi.
*  *Installing ROS Indigo on RPi* : [Install ROS](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi). Deviations from the steps in link :
 * No need for applying patch to collada_urdf 
 * Didn't apply a patch for RViz as it is not needed at this stage
 * Robot/Perception weren't installed 
 * In the last step, code was built using -j2 option instead of -j4 (to save RAM)
* *USB locking settings* : Change the ATTRS{devpath} parameter in 72-micro-devel.rules accordingly for the same

## Debugging 

* **ROSlaunch Error** *Can't locate launch node of type<>* : make python file executable by chmod +x filename.py and putting `#!/usr/bin/env python` in the start of python file
* **ValueError** : *The channel sent is invalid on a Raspberry Pi* : Change board mode from GPIO.board to GPIO.BCM
* **Drive Jerks** : GPIO.cleanup() **MUST** be always done for drive w/o jerks

## Deadlines
- [x] Steer sorted on laptop 
- [x] Installed ROS on RPi
- [x] Drive on Rpi by 8/12
- [x] Test of steer on Rpi by 9/12
- [ ] Ground testing of mobility 
