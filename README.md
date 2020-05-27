# autobot
Autonomous Vehicle Project


### camera
OpenMV H7 Camera

#### Website

The website is the best source of information, and anything you might need.https://openmv.io/

#### Openmv H7 Cam

Cam itself is easiest to play with using openmv IDE https://openmv.io/pages/download

#### Host side

Need micropython library. Git repository here: https://github.com/micropython/micropython
Wiki page of git repository contains helpful instructions for installation


#### Running

upload camera code using USB cable, modify main.py. Main.py will be the script that is run on startup for the camera.


Run host code in normal python. Might need to modify USB port code to find correct port.
 
### motorcontroller


#### hardware 

Connect the Arduino to the Raspberry Pi via USB

#### software 
Upload the .ino file into your arduino. Make sure to download the Adafruit Motor library.
Load the python script "MotorController.py" into your rasberry pi, into the same directory as your python code that will give out movement commands. 


#### calibration

You shouldn't ever need to to change the .ino file, but the python file uses experimentally-derived constants for the movement, which may be different for your bot. The test code is designed to have the bot turn 90 degrees right twice, then back 90 degrees left twice, and then go forward two meters, each action with a one second delay. But if the bot doesn't do the above, i.e. it turns too little or too much or moves too far, you'll need to change the constants. If so change the 'turnRatio' and 'moveRatio' parameters until the bot does the above. Roughly you should be able to adjust each with the following process:

moveConstant:

1. Measure actual distance covered in meters.
2. Divide 2 meters by the actual distance.
3. Multiply previous moveConstant by this to get the new moveConstant.

turnConstant: 

1. Measure actual right angle turned in degrees.
2. Divde 180 degrees by actual angle.
3. Multiply previous turnConstant by this to get the new turnConstant. 

runOnStartup:

1. On command line execute 'sudo nano /etc/rc.local'
2. Erase IP address logging code (default) and replace with 'python /home/pi/re$
3. Make sure the ampersand is there (it tells the Pi to continue loading progra$
4. Ctrl+X will save, Y, and enter will get you back to command line

### IMU

1. https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts
2. Connect the SCL and SDA pins to the correct pinouts on the Pi (and also power and ground) ... only these 4 wirings are needed

3. https://github.com/fm4dd/pi-bno055
4. Enable I2C on your Pi (Pi > Preferences > Raspberry Pi Configuration > Interfaces)  
5. Clone this repository. Follow instructions in README. Once you "make", the sensors should be calibrated
6. From there a command like "./getbno055 -t eul" will give orientation info (90deg test is working)
Make sure the mode (-m) has fusion set to ON. If not, execute command "./getbno055 -m ndof" and then you can get readings with "./getbno055 -t eul"
7. Once all of this is setup and functional, see imu_integrated_movement (specifically the getYaw() method) to see how to obtain IMU information from within another python script

Notes on IMU:

1. Be sure to remove pin from IMU's Vin power and replug it every new trial
2. IMU's "zero-direction" is the direction it is facing at the moment it receives power
3. Known bug with the call to IMU from within a python script: Error with subprocess. It's random and can be fixed by making the call to IMU within a while loop with try/except. Please see MarcDemo for reference in GetYaw()


### DWM

DWM1001 Positioning System

#### Files

* DWMTag.py: Class that defines the Tag object. Initializing the tag opens the serial connection and issues commands to begin reading the position of the Tag. Remaining methods are used to update the stored position, get the position, and close the serial connection.

* DWM1001_apicommands.py: Contains the commands listed in the DWM1001 api guide in a convenient place so that they are ready to be used in any program.

* TagClassTest.py: A simple test that initializes an instance of DWMTag, and demonstrates how to use the class methods to obtain position readings from the Tag.


#### How to Run

Prerequisites: 
* A properly configured network of DWM1001s, with all anchors powered on. See https://www.decawave.com/wp-content/uploads/2019/03/DWM1001_Gateway_Quick_Deployment_Guide.pdf
* pyserial library (install by running 'pip install pyserial')
* While this code is compatible with both Python 2 and 3, we recommend using Python 3

1) Plug the DWM1001 dev board configured as the Tag into the USB port of the Raspberry Pi (or pc).
2) Find the port name for the device. On the pi, this can be done by running 'dmesg' in terminal. On a Mac, run 'python -m serial.tools.list_ports -v'
3) Run TagClassTest, specifying port_name when the tag is initialized.
4) The tag should connect and successfully print 200 position readings before closing the serial connection.

#### Suggestions

For multithreading, call Tag.update_position() in a while loop. This continuously updates Tag.x_position, Tag.y_position and Tag.curr_time by reading the data coming in over the serial connection.

Only call Tag.get_pos() to get [Tag.x_position, Tag.y_position] which returns the values of Tag's instance variables.

#### Links
For the complete guide to the DWM1001 API: https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf

For the DWM1001 quick deployment guide: https://www.decawave.com/wp-content/uploads/2019/03/DWM1001_Gateway_Quick_Deployment_Guide.pdf


