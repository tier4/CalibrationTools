ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# hadolint ignore=DL3008
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace

# TODO: restore upstream
RUN mkdir -p /workspace/src && wget --progress=dot:giga https://raw.githubusercontent.com/amadeuszsz/CalibrationTools/feat/jazzy-support/calibration_tools_standalone.repos --directory-prefix=/workspace

RUN vcs import src < calibration_tools_standalone.repos

RUN rosdep install -y --from-paths "$(colcon list --packages-up-to sensor_calibration_tools -p)" --ignore-src

RUN source /opt/ros/"${ROS_DISTRO}"/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to sensor_calibration_tools

RUN echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc
