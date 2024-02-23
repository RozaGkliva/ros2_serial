# ROS 2 package for interfacing serial devices

Requirements:
|Ubuntu 20.04 LTS (Focal Fossa)| ROS 2 Foxy Fitzroy |
|:---:|:---:|


# Download

### Clone the repository to your ROS 2 workspace (e.g., ```ros2_ws/```)
  ```
  cd ~/ros2_ws/src
  git clone https://github.com/RozaGkliva/ros2_dev_wip.git
  ```

### Edit the parameters in the configuration file
  ```
  ~/ros2_ws/src/ros2_dev_wip/config/serial.yaml
  ```

### Build with colcon.
  ```
  cd ~/ros2_ws/
  colcon build --packages-select ros2-dev_wip --symlink-install
  ```


# Usage

  ```
    ros2 launch ros2_dev_wip serial_interface.launch.py
  ```


# Roadmap
- [ ] Write script that finds the serial port
- [ ] Add Arduino example (check: using parameters from yaml to set up communications - https://arduinojson.org/)
- [ ] .amentignore in Arduino directory
