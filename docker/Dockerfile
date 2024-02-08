
FROM osrf/ros:humble-desktop-jammy

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN mkdir -p /workspace/src

RUN wget https://raw.githubusercontent.com/knzo25/CalibrationTools/feature/new_api_documentation/calibration_tools_standalone.repos --directory-prefix=/workspace

RUN vcs import src < calibration_tools_standalone.repos

RUN cd /workspace && rosdep install -y --from-paths `colcon list --packages-up-to sensor_calibration_tools -p` --ignore-src

RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to sensor_calibration_tools

RUN echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc
