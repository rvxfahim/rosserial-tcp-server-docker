FROM ros:noetic-ros-base


RUN apt-get update 
RUN apt install --no-install-recommends -y wget apt-transport-https unzip ros-noetic-rosserial-python supervisor bash

RUN mkdir -p /rosstart
RUN mkdir -p /rosstart/log/
COPY health-check.sh /rosstart
COPY entrypoint.sh /rosstart
COPY start-roscore.sh /rosstart
COPY start-rosserial.sh /rosstart
RUN chmod +x /rosstart/entrypoint.sh
RUN chmod +x /rosstart/start-roscore.sh
RUN chmod +x /rosstart/start-rosserial.sh
RUN chmod +x /rosstart/health-check.sh
RUN mkfifo /rosstart/start-roscore.pipe


# Create directories for Supervisor's UNIX socket and log files
RUN mkdir -p /var/run/supervisord /var/log/supervisor

# Copy the Supervisor configuration file to the container
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

ENTRYPOINT ["sh","/rosstart/entrypoint.sh"]