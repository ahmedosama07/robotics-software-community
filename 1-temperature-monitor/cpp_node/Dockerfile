FROM ros:humble-ros-base-jammy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /workspace

# Copy source files
COPY cpp_node/ /workspace/src/temperature_monitor/

# Copy message interfaces
COPY msg_interfaces /workspace/src/temperature_interfaces/

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select temperature_interfaces && \
    colcon build --packages-select temperature_monitor --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]