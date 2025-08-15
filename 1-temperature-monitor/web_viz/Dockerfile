FROM ros:humble-ros-core

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-dev \
    build-essential \
    postgresql-client \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    aiohttp==3.8.* \
    websockets==11.* \
    numpy \
    psycopg2-binary

# Create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy message interface first
COPY msg_interfaces/ /workspace/src/temperature_interfaces/

# Build message interfaces first
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /workspace && \
    colcon build --packages-select temperature_interfaces && \
    echo 'Message interfaces built successfully'"

# Copy web visualizer source
COPY web_viz/src/ /workspace/src/

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Create a test script to verify setup
RUN echo '#!/bin/bash' > /workspace/test_setup.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /workspace/test_setup.sh && \
    echo 'source /workspace/install/setup.bash' >> /workspace/test_setup.sh && \
    echo 'python3 -c "import aiohttp; print(\"aiohttp:\", aiohttp.__version__)"' >> /workspace/test_setup.sh && \
    echo 'python3 -c "import websockets; print(\"websockets:\", websockets.__version__)"' >> /workspace/test_setup.sh && \
    echo 'python3 -c "from temperature_interfaces.msg import TemperatureReading; print(\"ROS messages: OK\")"' >> /workspace/test_setup.sh && \
    echo 'echo "Checking if template files exist:"' >> /workspace/test_setup.sh && \
    echo 'ls -la /workspace/src/templates/ 2>/dev/null || echo "Templates directory not found"' >> /workspace/test_setup.sh && \
    echo 'ls -la /workspace/src/static/ 2>/dev/null || echo "Static directory not found"' >> /workspace/test_setup.sh && \
    chmod +x /workspace/test_setup.sh

# Expose ports
EXPOSE 8080 8081

# Set the working directory
WORKDIR /workspace

CMD ["/bin/bash"]