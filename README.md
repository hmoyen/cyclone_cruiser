# User Manual
## Cyclone Cruiser

## Introduction

Welcome to the Cyclone Cruiser user manual. This document contains the necessary instructions to connect and control the cart, as well as a description of its main features. Follow the steps to use the cart efficiently.

## Objective

The Cyclone Cruiser is a remote-controlled cart prototype for indoor mapping. Its purpose is to identify the location of obstacles around it and, based on this information, create a map of the environment. 

Being remotely controlled, it can access locations that are inaccessible to humans or be controlled from a remote control cabin away from the cart. Its applications range from large-scale warehouse mapping to the recognition of unknown areas, such as caves and debris from collapses, potentially assisting search teams in rescue operations.

## Components

The Cyclone Cruiser consists of:

- A DE0-CV FPGA development board
- Two ESP32 boards to support Bluetooth and MQTT, mounted on two breadboards along with a voltage conversion chip
- A free wheel and two motors connected to two torque wheels
- An H-bridge for controlling the motor power via PWM
- A powerbank to power the system

## Initial Setup

Before starting, ensure you have the following items:

- A smartphone or tablet with Bluetooth enabled
- The ArduinoCar app installed (available in app stores for Android)
- The Cyclone Cruiser charged and turned on
- A computer with the necessary software installed (Intel Quartus Prime, Arduino IDE, and other software related to the graphical interface described below)

Once you have these items, download the FPGA firmware from the link: [Projeto_Cyclone_FPG](https://github.com/hmoyen/LabDigII/blob/main/Cyclone_Cruiser/Semana5/qar/cyclone.qar). Using Intel Quartus Prime, compile the project and program it on the DE0-CV development board via a USB-Blaster cable.

**IMPORTANT:** Whenever the FPGA board is turned off, it loses the programming, so it needs to be reprogrammed each time it is restarted.

Once this is done, the Cyclone Cruiser should be fully programmed, as the ESP32s should already be pre-programmed. However, if needed, you can reprogram the two ESP32s with the codes available at these links:
[ESP32_Bluetooth](https://github.com/hmoyen/LabDigII/blob/main/Cyclone_Cruiser/Semana5/esp/esp_bluetooth) and [ESP32_Interface](https://github.com/hmoyen/LabDigII/blob/main/Cyclone_Cruiser/Semana5/esp/calculate_esp.cpp). Use Arduino IDE with a Micro-USB cable to program the files on the ESP32 positioned at the center of the breadboards and at the end of the breadboards, respectively.

## Connecting to the Cart

1. Turn on the Cyclone Cruiser and enable Bluetooth on your mobile device.
2. In the ArduinoCar app, select the option to search for Bluetooth devices.
3. Choose the device identified as **Carrinho1** from the list and connect.
4. Wait for the connection to be confirmed by the app.

## Controlling the Cyclone Cruiser

Once the connection is established, you can control the cart through the app interface.

### Speed Control
- Press the **Y** button to increase speed.
- Press the **X** button to decrease speed.
- There are 4 available speed settings.

### Direction Control
- Use the left directional arrows to move the Cyclone Cruiser:
  - Left/Right arrows: turn the cart left or right.
  - Up/Down arrows: move the cart forward or backward.

## Tools Required

Before you begin, it is important to understand the tools involved:

- **ROS 2 (Robot Operating System):** A widely used middleware for creating modular and scalable robotic systems.
- **Gazebo Harmonic:** An advanced 3D simulator to test robots in virtual environments before deploying them in the real world.
- **RViz 2:** A graphical visualization tool that allows you to monitor the state of the robot, sensors, and other system components in real-time.

These open-source tools facilitate integration, reduce costs, and promote collaboration in the development of robotic systems.

## Installation and Setup

To set up the environment, follow these steps:

### 1. Clone the Repository
Clone the project repository using the `--recursive` flag:
```bash
git clone --recursive https://github.com/hmoyen/cyclone_cruiser.git
```

## 2. Install ROS 2, Gazebo Harmonic, and RViz 2

In the cloned repository, run the installation script:

```bash
cd cyclone_cruiser
bash install.sh
```

## 3. Compile the Workspace

After installing the dependencies, compile the workspace:

```bash
cd interface
cd src/mqtt_client
git checkout ros2
cd ../../..
colcon build
```

## 4. Configure the ESP32

1. Make sure the cart is powered on and connected to the battery.
2. If outside the lab, configure the correct WiFi in the code  (https://github.com/hmoyen/LabDigII/blob/main/Cyclone_Cruiser/Semana5/esp/esp_final.cpp.)
3. Update the broker IP to your local broker address.
4. Load the code onto the ESP32 using the Arduino IDE.

## 5. Test MQTT

Once the setup is complete, you can check its functionality:

```bash
mosquitto_sub -h localhost -t "sonar1"
```

This command should show continuous messages published by the sonar sensor at a frequency of 5 Hz.

## 6. Set Up the Bridge with ROS 2

Enable the bridge between MQTT and ROS 2:

```bash
cd interface
source install/setup.bash
ros2 launch mqtt_client standalone.launch.ros2.xml
```

To verify, use the following command:

```bash
ros2 topic echo /sonar1
```

The messages displayed in the terminal should match exactly what is seen in the MQTT topic, confirming that the MQTT-ROS 2 bridge is working correctly. To build the map, run https://github.com/hmoyen/cyclone_cruiser/blob/main/interface/scripts/calc_acc.py in the scripts folder:

```bash
cd interface/scripts
python3 calc_acc.py
```

Additionally, the RViz 2 interface will automatically open.

To view the map being built in real-time:

1. In RViz 2, click `File > Open Config`.
2. Navigate to the `rviz` folder within the `interface` package.
3. Select the `config.rviz` file.

After this setup, the map will be displayed in the RViz 2 interface, allowing you to monitor the data transmitted by the robot in real-time or when simulating test data. The result can be seen in Figure \ref{fig:rviz}. If you only want to test the interface, to ensure it is working properly, run the `simulate.py` code, along with `calculate.py`.

## Data Storage with ROSBag

The rosbag tool is used in ROS to record and replay topic messages. This is particularly useful for post-experiment data analysis.

To record all system messages into a rosbag file, use the following command:

```bash
ros2 bag record -a
```

The generated file can be replayed for analysis and debugging without needing to run the robot or simulator again. To replay it, run:

```bash
ros2 bag play <bag_file>
```

## Testing the System

With all components configured:

1. Turn on the cart and connect it to the configured WiFi network.
2. Monitor the data in the MQTT or ROS 2 topics.
3. Use RViz 2 to visualize the interface and the robot's position in real-time.

## Future Applications - TODO

In the `gazebo` folder, there is an SDF model of the cart, which aims to be a 3D digital twin of the real robot. The integration of the model into Gazebo is still under development but can be tested soon using the code `twin.py`. To test the Gazebo simulation:

1. Open the model in Gazebo with the following command:

```bash
gz sim robot.sdf
```

1. Then run the `twin.py` script. The model in Gazebo should start to mimic the movements observed in the RViz map.

Note that the robot’s speed control in the simulation may need improvement. This adjustment is up to the developer if they wish to implement improvements in this aspect. Additionally, to ensure proper communication between Gazebo and ROS 2, run the bridge between the two systems with this command:

```bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

This will allow data exchange between Gazebo and ROS 2, enabling the digital model in Gazebo to behave similarly to the real robot.
