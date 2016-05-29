Here we describe how to compile and use the source codes in this directory (csp3).
Descriptions of each code are provided at the beginning of each source file. 
Here csp3 stands for Create serial-port packet processor.

The source codes are written to demonstrate an example of how to use the iRobot 
Create robot using a usb-to-serial cable. In this example, the robot learns a 
control task using the sarsa lambda algorithm. To make sure the robot remains 
within a particular bounded region, we assume that the color of the target 
region is different than the color of the outside region. The robot has four 
cliff sensors pointing downward, using which the change in the color of the two
regions is detected (e.g. between black and yellow), and the robot is kept within 
the target region. However, the code can be used just as an example of serial 
communication to the robot without understanding this concept of colored regions
as well. The main code will still make the robot move when executed.

The main code is 'sarsa.c'. It is an example of how to set up a connection with 
the iRobot Create robot, collect data from the robot as fast as possible and at 
the same time use a learning agent that runs on a different time cycle. It should 
be compiled in the following way:

 cc sarsa.c -pthread -o sarsa

For mac os x, '-pthread' can be omitted.

To execute, type:

 ./sarsa /dev/tty.something 

Here 'tty.something' refers to the serial port driver corresponding to the 
usb-to-serial cable.

This code requires a file named 'cliffThresholds.dat'. Here the cliff threshold 
values for the boundary of the target region are stored. An example file is 
already provided so that the program works right away.

Additional source codes are also given so that the robot can be re-calibrated to 
regions with different colors (e.g., a table or a carpet).

The code for calibrating the cliff values is 'cliffcalib.c'. 
To compile the code, type:

 cc cliffcalib.c -pthread -o cliffcalib

To execute, put the robot on the region where the robot must reside. Then type:

./cliffcalib /dev/tty.something > inside

The robot will wiggle for 10 seconds, and the minimum and maximum values of the 
four cliff sensors will be stored in the file named 'inside'. Any other file name 
can also be used. Repeat the same by putting the robot on the other region which 
defines the outside of the target region. Store the data in a file, named e.g. 
'outside'.

The source file named 'makethresh.c' takes the cliff values from the two files 
and makes the threshold values. Compile it in the following way:

 cc makethresh.c -o makethresh

To execute, type:

 ./makethresh inside outside

The order in which the file names are given matters. The first one is treated as 
the target region of the robot. This execution produces the 'cliffThresholds.dat' 
file.
 
Serial cables:
--------------
For cable-based serial port communication to the Creates from a Mac, you will need to
install drivers.
		
For the StarTech serial port cable Install the driver for Prolific/PL2303X/MacOSX10.6/ 
from the CD in the box

For the turtlebot USB-Create cable go to 
http://www.ftdichip.com/Drivers/VCP.htm
Install the 64 bit OSX driver.



