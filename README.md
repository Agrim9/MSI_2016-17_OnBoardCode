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

