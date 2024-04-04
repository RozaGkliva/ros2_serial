# ROS 2 package for interfacing serial devices

# Requirements
- Ubuntu 20.04 LTS (Focal Fossa)
- ROS 2 Foxy Fitzroy or later
- install appropriate serial library
  - for cpp: install serial interface library (*NB! c++ seems more stable and fast*)
    ```
    git clone https://github.com/RoverRobotics-forks/serial-ros2
    cd serial-ros2/
    make
    make install
    cd ~/dev_ws/ && colcon build --packages-select serial
    ```
  - for python: install pyserial from PyPI 
    ```
    python3 -m pip install pyserial
    ```



# Download

  Clone the repository to your ROS 2 workspace (e.g., <em>"ros2_ws/"</em>)

  ```
  git clone https://github.com/RozaGkliva/ros2_serial.git
  ```

# Usage
<ol>
  <li>Edit the parameters in the configuration file

  ```
  ~/ros2_ws/src/ros2_serial/config/serial.yaml
  ```

  to find the port of your device run
  ```
  python3 -m serial.tools.list_ports -v
  ```
  </li>

  <li>Build the packages with colcon.

  ```
  cd ~/ros2_ws/
  colcon build --path ~/<path_to_package>/ros2_serial/* --symlink-install
  ```
 </li>

  <li>

  With your device connected, run the interface:
  ```
  ros2 launch ros2_cppserial serial_interface.launch.py
  ```
  or
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
- [ ] c++ version works better and faster. Consider removing the python version
- [x] license got messed up when the repository structure changed. TODO: FIX!
- [x] Hydromasts have set serial ports, gps does not but has a serial hardware id
- [ ] Clear buffer until recent message appears (there seem to be some old/incomplete packets in the buffer)
