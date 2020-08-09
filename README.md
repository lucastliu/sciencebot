<!-- PROJECT SHIELDS -->
<!--
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

[![GPLv3 License][license-shield]][license-url]



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/lucastliu/sciencebot">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">sciencebot</h3>

  <p align="center">
    A modular low-cost research vehicle platform
    <br />
    <a href="https://github.com/lucastliu/sciencebot"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/lucastliu/sciencebot">View Demo</a>
    ·
    <a href="https://github.com/lucastliu/sciencebot/issues">Request Feature</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
## Table of Contents

- [Table of Contents](#table-of-contents)
- [About The Project](#about-the-project)
- [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
    + [Raspberry Pi](#raspberry-pi)
    + [Python 3](#python-3)
    + [ROS2 Eloquent Elusor](#ros2-eloquent-elusor)
  * [Installation](#installation)
- [Usage](#usage)
  * [Spin up](#spin-up)
  * [turtlesim basic](#turtlesim-basic)
  * [turtlesim PID control](#turtlesim-pid-control)
  * [sciencebot](#sciencebot)
- [Helpful Tips](#helpful-tips)
- [Resources](#resources)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

#### Raspberry Pi

This project was developed on a Raspberry Pi 3B+ running Raspian Buster. Setups that use other Pi hardware or OS versions, or even other Linux devices, will likely operate in a highly similar fashion, but not in the exact same manner.

1. Storage

    Running with Raspbian Buster, the minimum SD card size is 8 GB. However, running this project will require more resources, and a 16 GB card was found to be inadequate as well. A `64 GB micro SD` was used in the build, and is easily sufficient.

2. Virtual Memory

    Due to the limited abilites of an RPi, during installation, it is often useful / necessary to leverage [swap space](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/5/html/deployment_guide/ch-swapspace) . This can be achieved by editing the file `/etc/dphys-swapfile`, increasing the `CONF_SWAPSIZE` variable (`1024` seems to be sufficient). Full discussion for RPi 3 B+ can be found [here](https://www.raspberrypi.org/forums/viewtopic.php?t=211804)

3. Remote Access

    This step is optional, but highly recommended. The Pi supports HDMI output, but a wired connection is quite cumbersome, especially for a moving vehicle. There are several options for remote access, one of which is as follows.

    Virtual Network Computing (VNC) will allow one to view and control the Pi from another device wirelessly. Raspbian comes with a native / pre-installed VNC. Full instructions here: [VNC](https://www.raspberrypi.org/documentation/remote-access/vnc/)

    
    You will want to [setup a static IP Address](https://pimylifeup.com/raspberry-pi-static-ip-address/) as well.

    If the resulting resolution on your host device does not look right, you may need to [adjust the resolution](https://help.realvnc.com/hc/en-us/articles/360002249917-VNC-Connect-and-Raspberry-Pi#troubleshooting-vnc-server-0-7).


#### Python 3
This project requires Python 3. This should be available by default on the Pi with Raspbian Buster.

#### ROS2 Eloquent Elusor

Your primary and best reference is the [ROS2 website](https://index.ros.org/doc/ros2/)

Their [tutorials](https://index.ros.org/doc/ros2/Tutorials/#tutorials) are also very informative, and a great place to start at. Several of the beginner example packages have been provided.

This project was built for ROS2 Eloquent: [Instructions for installing from source](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Development-Setup/)

The build process may require several attempts, as ROS does not have Tier 1 support for the Pi. In order to reduce the strain on the Pi, add the following flag modifications to build commands:

```sh
MAKEFLAGS="-j1 -l1" colcon build  --executor sequential
```

### Installation
 
1. Clone sciencebot
```sh
git clone https://github.com/lucastliu/sciencebot.git
```
2. [Create an ROS workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) under dev_ws. Pay special attention to sourcing the overlay and underlay.

3. Build the project
```sh
cd .../dev_ws/src
```
```sh
colcon build --symlink-install
```

<!-- USAGE EXAMPLES -->
## Usage

### Spin up

Open a terminal and navigate to the project
```sh
cd .../dev_ws
```

Source the overlay

```sh
. install/setup.bash
```

### turtlesim basic

Run this first to confirm that the software has installed correctly.
Turtlesim is also a useful simulation for developing basic control theory in a controlled, simplified environment.

Follow the Spin up steps.

```sh
ros2 run turtlesim turtlesim_node
```
You should see a GUI pop-up with a turtle if project installation was successful. Close the simulation and window.

### turtlesim PID control

Control the turtle using PID goal seeking.

Complete the Spin up steps.

Run the launch file
```sh
ros2 launch nav turtlesim.launch.py
```

Wait for indication that "Turtle PID Node is Live"

In a seperate terminal, follow the Spin up steps and run

```sh
python3 ./src/nav/nav/clients/tune_position_pid_client.py
```

You will be prompted for a goal position X Y, as well as PID parameters.

Here are some good defaults:

```sh
Desired X Y: 6 7
Enter linear PID Constants : 1.5 0 0
Enter Angular PID Constants : 6 0 0
```

If you have chosen good parameters, the turtle will move to the desired setpoint.


### sciencebot



## Helpful Tips


## Resources
[ROS2](https://index.ros.org/doc/ros2/)

[ROS answers](https://answers.ros.org/questions/)

[r/ROS](https://www.reddit.com/r/ROS/)

[Raspberry Pi Forums](https://www.raspberrypi.org/forums/)


<!-- CONTRIBUTING -->
## Contributing

Contributions are key to any open source project, and are definitely welcome here!

1. Fork the Project, and clone it (`git clone git@github.com:YOUR_USERNAME/sciencebot.git`)
2. Create your Feature Branch (`git checkout -b FEATURE_NAME`)
3. Commit your Changes (`git commit -m 'INFORMATIVE IMPROVEMENT MESSAGE'`)
4. Push to the Branch (`git push origin FEATURE_NAME`)
5. Open a Pull Request



<!-- LICENSE -->
## License

    Copyright © 2020 Lucas Liu

    sciencebot

    sciencebot is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    sciencebot is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with sciencebot.  If not, see <https://www.gnu.org/licenses/>.

This repository includes modified versions of other source code from the ROS open source community. The relevant subfolders each have their own license. All other files fall under the GPLv3 license as described above.

<!-- CONTACT -->
## Contact

Your Name  - lucas.liu@duke.edu

Project Link: [https://github.com/lucastliu/sciencebot](https://github.com/lucastliu/sciencebot)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

Special thanks to Dr. Tyler Bletsch for his mentorship during this project



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[license-shield]: https://img.shields.io/badge/License-GPLv3-blue.svg
[license-url]: https://github.com/lucastliu/sciencebot/LICENSE.txt
[product-screenshot]: images/screenshot.png
