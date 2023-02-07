# Bluetooth Sensors for the Science Classroom

![](/Media/IntroBanner.png)

Inspired by the CLEAPSS Arduino build guides I have been working on ESP32 based devices with off the shelf sensors to help in the teaching of science. Thanks to a school grant from the Institute of Physics (IOP) I have now started working with students in an extra-curricular club to give them a chance to participate in the development of these devices.

This repository is a record of where I have got to at this stage. The students work in a seperate repo which we will merge at some future date. Much of the wiki here is written with students, and any teachers who wish to work with students in a similar manner, in mind.

The challenge I have given the students:

**Aim : Designing and building a collection of wireless sensors to be used in the science classroom.**
* Identifying sections of the science curriculum that would be enhanced by the use of live data visualisation
* Selecting sensors / electronics to achieve the data collection
* Designing and 3D printing cases and accessories for the devices
* Using Arduino IDE to code the data collection / data processing by the sensor and interface with the phyphox app

**Design Principles**
* Choosing components that allow for minimal soldering or wiring
* Case design that means limited screws / use of tools
* Choosing readily available components (in the first instance from UK based suppliers - but should be true for Europe and N America)

## Progress so far

### **Distance Sensor**
![](/Media/DistanceSensorBanner.png)

Based on ST's VL53L1X time of flight sensor this device gives distance against time measurements from 0 to ~180cm  at a data rate of up to 50 Hz (max range higher at lower data rate). Also outputs velocity and acceleration against time.

There is an implementation of a 'Walk the Line' game based on code written by Dominik Dorsel (https://github.com/Dorsel89/WalkTheGraph). This allows you to challenge students to move in such a way to create a distance time graph that matches a pre-drawn line on the graph. They are scored by how long they spend on the line with a number of lines to choose from.

Navigate to the [wiki](https://github.com/RGill-1/Bluetooth-Science-Sensors/wiki/4.-Distance-Sensor) page for the Build Guide and example experiments.

### **Volt/Ammeter**
![](/Media/VASensorbanner.jpg)

Uses an INA219 chip to measure current and a seperate ADS1015 ADC with potential divider onboard for potential difference. This device is capable of simultaeously measuring those quantities and plotting them against time and/or each other. Resolution of 1mA and 10mV respectively with a range of 0-3.2A and 0-18V. Data rate of ~100Hz.

Navigate to the [wiki](https://github.com/RGill-1/Bluetooth-Science-Sensors/wiki/5.-Volt-Ammeter) page for the Build Guide and example experiments.

### **Carbon Dioxide Sensor**
![](/Media/CO2SensorBanner.jpg)

This device uses the Sensiron SCD41 photoacousitic sensor to detect CO2 levels within a range of 400 - 5,000 ppm. With a data rate of one reading every 5s and a manufacturer claimed accuracy of +/- (40ppm + 5% of reading).

Navigate to the wiki page for the Build Guide and example experiments.

### **Light Sensor**

An onboard VEML7700 light sensor detects light intensity between 0 - ~120,00 lux. This sensor uses the same case as the distance sensor with a new lid and component tray to be printed.

Navigate to the [wiki](https://github.com/RGill-1/Bluetooth-Science-Sensors/wiki/7.-Light-Sensor) page for the Build Guide and example experiments. 

### **Force Sensor**

A 5kg load cell and a 24bit ADC allow us to measure force against time. 

Navigate to the wiki page for the Build Guide and example experiments.

### **9 DoF Accelerometer**

Using a LSM6DSOX+LIS3MDL 9 degrees of freedom IMU that combines accelerometer, gyro and magnetometer in one package this device gives full motion data. 

Navigate to the wiki page for the Build Guide and example experiments.

## Future projects

### **Hybrid Sensors**
Combining two or more of the above sensors together to create hybrid sensors - eg CO2 with light intensity for photosynthesis or distance and light intensity for inverse square law - would allow for more phenomena to be demonstrated to students. This is made easier / cheaper by the fact we have not soldered components in our devices so the sensor components can be readily swapped from one device to another.

### **Pressure Sensor**
The barometer in mobile phones used natively in phyphox is very effective at demonstrating effects such as the change in pressure with altitude. With a waterproof pressure sensor we could show how pressure changes with depth as well. With onboard temperature sensors the Gas Laws would be easily demonstrated.

### **Colorimetry**

### **Temperature Probes**

### **Magnetometer**
