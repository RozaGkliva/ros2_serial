# ROS 2 package for interfacing serial devices

# Requirements
- Ubuntu 20.04 LTS (Focal Fossa)
- ROS 2 Foxy Fitzroy or later
- for pyserial: install from PyPI
  ```
  python3 -m pip install pyserial
  ```
- for cpp: install serial interface library LibSerial
  ```
  sudo apt install libserial-dev
  ```



# Download
<ol>
  <li>Clone the repository to your ROS 2 workspace (e.g., <em>"ros2_ws/"</em>)

  ```
  cd ~/ros2_ws/src
  git clone https://github.com/RozaGkliva/ros2_dev_wip.git
  ```
  </li>

  <li>Edit the parameters in the configuration file

  ```
  ~/ros2_ws/src/ros2_dev_wip/config/serial.yaml
  ```

  to find the port of your device run
  ```
  python3 -m serial.tools.list_ports -v
  ```
  </li>

  <li>Build the package with colcon.

  ```
  cd ~/ros2_ws/
  colcon build --packages-select ros2-dev_wip --symlink-install
  ```
  </li>
</ol>

# Usage

  With your device connected, run the interface:
  ```
  ros2 launch ros2_dev_wip serial_interface.launch.py
  ```


# Roadmap
- [ ] ~~Write script that finds the serial port~~ A script is not necessary. Added command in guide.
- [ ] Add Arduino example (check: using parameters from yaml to set up communications - https://arduinojson.org/); use .amentignore in Arduino directory
