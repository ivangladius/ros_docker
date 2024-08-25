# Use the official ROS Noetic image as a base
FROM ros:noetic-ros-core

# Install necessary packages
RUN apt-get update && apt-get install -y \
    openssh-server \
    sudo \
    iproute2 \
    iputils-ping \
    net-tools \
    curl \
    wget \
    git \
    vim \
    neovim \
    ranger \
    htop \
    tmux \
    build-essential \
    python3 \
    python3-pip \
    jq \
    dnsutils \
    traceroute \
    tcpdump \
    nmap \
    telnet \
    netcat \
    rsync \
    lsof \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Set up SSH server
RUN mkdir /var/run/sshd \
    && echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config

# Set the working directory to root's home directory
WORKDIR /root

# Copy the setup script
COPY setup_catkin.sh .

# Copy SSH keys
COPY .ssh /root/.ssh
RUN chmod 600 /root/.ssh/id_rsa && chmod 644 /root/.ssh/id_rsa.pub && chmod 644 /root/.ssh/known_hosts

# Run the setup script
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && /root/setup_catkin.sh"

# Expose SSH port
EXPOSE 22

# Start SSH server
CMD ["/usr/sbin/sshd", "-D"]