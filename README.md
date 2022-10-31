# object_detection

## Introduction

To follow the detected object with the drone camera. In the project, we performed the tracking process by converting the position values of the detected object into the coordinate system using the YOLOv3 algorithm and the MOSSE tracking algorithm.

## Dependencies
Python3, numpy, opencv, dronekit

## Getting started

1. You need to install dronekit library using pip:

   ```
   pip install dronekit
   ```

2. You can install the dronekit-sitl library on all platforms using pip:

   ```
   pip install dronekit-sitl -UI
   ```

3. If you can see drone tracking operations on the map in Mission Planner, you can install the Mission Planner program for this:

   <a href="https://ardupilot.org/planner/docs/mission-planner-installation.html">Click here</a> to download the mission planner.

4. you need install opencv library using pip:

   ```
   pip install opencv-python
   ```

5. you need install numpy library using pip:

   ```
   pip install numpy
   ```

6. To establish the connection between the mission planner and the dronekit-sitl interface, we need to download the mavproxy api using pip:

```
pip install mavproxy pymavlink --user --upgrade
```
## Run Project

run the following command to run the code:

1. Run dronekit-sitl

   ```
   dronekit-sitl copter
   ```

2. Run mavproxy 

   ```
   link add 127.0.0.1:14550
   ```

3. Run mission planner

   Open the mission planner application, press the connect button, and then enter the port number to connect.

4. Run python project

   By running the python project and running the code by writing the live or video connection in the videoCapture method, the dronekit-sitl connection will be made.

