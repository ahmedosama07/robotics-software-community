FROM ros:humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    numpy \
    pytest-cov

# Set up workspace
WORKDIR /workspace

# Copy source files
COPY python_node/src/ /workspace/src/temperature_monitor/
COPY python_node/package.xml python_node/setup.py python_node/setup.cfg /workspace/src/temperature_monitor/

# Copy tests
COPY python_node/test/ /workspace/src/temperature_monitor/test/

# Create required resource directory and file
RUN mkdir -p /workspace/src/temperature_monitor/resource
RUN touch /workspace/src/temperature_monitor/resource/temperature_monitor

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
    colcon build --packages-select temperature_monitor

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