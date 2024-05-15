FROM ros:humble

# Set working directory
RUN mkdir /dev_ws
WORKDIR /dev_ws

RUN sudo apt update && sudo apt install -y -q --no-install-recommends ros-humble-rmw-cyclonedds-cpp

# Copy repo into the container
COPY ./src /dev_ws/src

# Set entrypoint by sourcing overlay workspace -> TODO: move to file and copy
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. \
    /dev_ws/install/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Build and test workspace
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
            colcon build"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
