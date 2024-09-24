
FROM osrf/ros:humble-desktop-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    wget

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace

RUN mkdir -p /workspace/src

RUN wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools_standalone.repos --directory-prefix=/workspace

RUN vcs import src < calibration_tools_standalone.repos

RUN rosdep install -y --from-paths `colcon list --packages-up-to sensor_calibration_tools -p` --ignore-src

RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to sensor_calibration_tools

RUN rm -rf /var/lib/apt/lists/*

RUN echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc
