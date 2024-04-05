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
  <li>Edit the parameters in the configuration file

  ```
  ~/<path to ros2_serial>/config/serial.yaml
  ```
  Parameters (with example values):
  ```yaml
  node_name:
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
    publish_topic:
      name: "hydromast_n26_data"
      frame_id: "n26"
  ```

  <!-- to find the port of your device you can run
  ```
  python3 -m serial.tools.list_ports -v
  ``` -->
  </li>

  <li> Edit the launch file
  
  ```
   ~/<path to ros2_serial>/ros2_cppserial/launch/serial_interface.launch.py
  ```
  </li>

  <li>Build the packages with colcon.

  ```
  cd ~/<path to workspace>
  colcon build --path ~/<path to ros2_serial>/* --symlink-install
  ```
 </li>

  <li>

  With your device connected, run the interface:
  ```
  ros2 launch ros2_pyserial serial_interface.launch.py
  ```

  </li>
</ol>

# Roadmap
- [x] ~~Write script that finds the serial port~~ A script is not necessary. Added command in guide.
- [ ] Add Arduino example (check: using parameters from yaml to set up communications - https://arduinojson.org/); use .amentignore in Arduino directory
- [x] Make node flexible with parameters in yaml file
- [ ] Published message type can be set in yaml, the actual message type can be defined in ros2_serial_interfaces, TODO: look into parsing the packet and populating the message (can I look into the message type to plan the parsing?)
- [x] c++ version works better and faster. Consider removing the python version
- [x] license got messed up when the repository structure changed. TODO: FIX!
- [x] Hydromasts have set serial ports, gps does not but has a serial hardware id
- [ ] Clear buffer until recent message appears (there seem to be some old/incomplete packets in the buffer)
