FROM ros:noetic-ros-base

RUN apt-get update 
RUN apt install --no-install-recommends -y wget apt-transport-https unzip ros-noetic-rosserial-python ros-noetic-rosbridge-suite supervisor bash dos2unix

RUN mkdir -p /rosstart
RUN mkdir -p /rosstart/log/
COPY health-check.sh /rosstart
COPY entrypoint.sh /rosstart
COPY start-roscore.sh /rosstart
COPY start-rosserial.sh /rosstart
COPY start-rostopic.sh /rosstart
COPY start-rosrun.sh /rosstart
COPY start-rosbridge.sh /rosstart
RUN chmod +x /rosstart/start-rosrun.sh
RUN chmod +x /rosstart/entrypoint.sh
RUN chmod +x /rosstart/start-roscore.sh
RUN chmod +x /rosstart/start-rosserial.sh
RUN chmod +x /rosstart/start-rostopic.sh
RUN chmod +x /rosstart/health-check.sh
RUN chmod +x /rosstart/start-rosbridge.sh
RUN mkfifo /rosstart/start-roscore.pipe
RUN mkdir -p /catkin_ws/src

# Navigate to the workspace
WORKDIR /catkin_ws/src

# Create a new ROS package
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_create_pkg my_package rospy std_msgs'

# Navigate back to the workspace root
WORKDIR /catkin_ws

# Build the ROS package
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Source the ROS package
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /catkin_ws/src/my_package/scripts
# Copy the Python script into the Docker image
COPY listener.py /catkin_ws/src/my_package/scripts/

# Make the script executable
RUN chmod +x /catkin_ws/src/my_package/scripts/listener.py
COPY CMakeLists.txt /catkin_ws/src/my_package/CMakeLists.txt
WORKDIR /catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Create directories for Supervisor's UNIX socket and log files
RUN mkdir -p /var/run/supervisord /var/log/supervisor

# Copy the Supervisor configuration file to the container
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

ENTRYPOINT ["sh","/rosstart/entrypoint.sh"]