A project involving mapping of an unknown, simulated hazardous environment using a tele-operated, mobile rescue robot.


<p align="center">
  <img src=Robot.png>
</p>


<b>Specifications</b>:

- Raspberry Pi 3B+ for communications, command processing, LIDAR hosting and power supply
- Arduino Uno for actuator controls (wheel motors, IR sensors, colour sensor)
- Ubuntu 16.0.4 Laptop with ROS platform for Hector SLAM (Simultaneous Localization & Mapping)
- Windows laptop with WSL (Windows Subsystem for Linux) terminal-based teleoperation of Robot

Features:
- TCP/IP with TLS for wireless communications between Pi and laptops
- UART between Pi and Arduino as well as Pi and Lidar
- Remote Arduino code reset using RPi GPIO
- Circuit implementation and cable management with extensive soldering
- WASD control using ncurses library
- Bare-metal and interrupt programming for Arduino-controlled subsystems
