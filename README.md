# ROS 2 package for interfacing serial devices

The serial interface node reads data from a serial port at a fixed rate, and publishes it as string with timestamp.
The following refers to the c++ package (ros2_cppserial) contained in this repository. As of March 2024 the python package (ros2_pyserial) is no longer updated as the c++ version was more stable and fast.

# Requirements
- Ubuntu 20.04 LTS (Focal Fossa) or later
- ROS 2 Foxy Fitzroy or later
- install serial library for c++
    <!-- - install serial interface library (*NB! c++ seems more stable and fast*) -->
    ```
    git clone https://github.com/RoverRobotics-forks/serial-ros2
    cd serial-ros2/
    make
    make install
    cd ~/dev_ws/ && colcon build --packages-select serial
    ```
    <!-- - for python: install pyserial from PyPI 
      ```
      python3 -m pip install pyserial
      ``` -->



# Download

Clone the repository to your ROS 2 workspace

  ```
  git clone https://github.com/RozaGkliva/ros2_serial.git
  ```

# Usage

<ol>
  <li>Create a yaml configuration file to declare the parameters and their attributes

  ```
  ~/<PATH_TO_THIS_REPO>/ros_cppserial/config/<device_name>.yaml
  ```
  Parameters (with example values):
  ```yaml
  node_name:
    ros__parameters:
      serial_params:
        use_port: False             # if True, the serial port will be used to connect to a device
        port: "/dev/ttyS2"          # if the port is fixed it should be used instead of device_hwid
        baud: 115200
        device_hwid: "0403:6001"    # if the port is not fixed the serial interface node will scan the ports for a device with a specific hardware id
      incoming_message:
        header: "N26"               # set if the packet contains a header
        footer: "/r/n"              # set if the packet contains a footer
        packet_size: 0
        rate_hz: 50                 # determines the rate of polling the serial port
      outgoing_message:
        is_single: False 
        is_continuous: False
        message: "r"                # specific to ATI FT sensor: "s" for continuous reading, "r" for single reading 
        rate_hz: 100                # 100 Hz
      publish_topic:
        name: "sensor_data"
        frame_id: "sensor_id"
  ```

  The node names should follow the following scheme: `<device_name>_interface`. When launching the nodes the same <device_name> will be used to create the node names and to look for parameter yaml files.

  <!-- to find the port of your device you can run
  ```
  python3 -m serial.tools.list_ports -v
  ``` -->
  </li>

  <!-- <li> Edit the launch file
  
  ```
   ~/<path to ros2_serial>/ros2_cppserial/launch/serial_interface.launch.py
  ```
  </li> -->

  <li>Build the packages with colcon.

  ```
  cd ~/<path to workspace>
  colcon build --path ~/<PATH_TO_THIS_REPO>/* --symlink-install
  ```
 </li>

  <li>

  With your devices connected, run the interface:
  ```
  ros2 launch ros2_cppserial multi_serial_interface.launch.py names:=device_names
  ```
  where 'device_names' is a comma-separated list of device names, e.g., 
  ```
  ros2 launch ros2_cppserial multi_serial_interface.launch.py names:='gps,axia80'
  ```
  will launch two nodes: gps_interface and axia80_interface, and will look for gps.yaml and axia80.yaml respectively.
  

  </li>
</ol>

# Roadmap
- [x] ~~Write script that finds the serial port~~ A script is not necessary. Added command in guide.
- [ ] Add Arduino example (check: using parameters from yaml to set up communications - https://arduinojson.org/); use .amentignore in Arduino directory
- [x] Make node flexible with parameters in yaml file
- [x] Published message type can be set in yaml, the actual message type can be defined in ros2_serial_interfaces, TODO: look into parsing the packet and populating the message (can I look into the message type to plan the parsing?)
- [x] c++ version works better and faster. Consider removing the python version
- [x] license got messed up when the repository structure changed. TODO: FIX!
- [x] Hydromasts have set serial ports, gps does not but has a serial hardware id
- [ ] Clear buffer until recent message appears (there seem to be some old/incomplete packets in the buffer)
