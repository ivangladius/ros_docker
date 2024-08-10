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

# Create a user for VS Code
RUN useradd -ms /bin/bash vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set up SSH server
RUN mkdir /var/run/sshd \
    && echo 'PasswordAuthentication yes' >> /etc/ssh/sshd_config \
    && echo 'vscode:vscode' | chpasswd

# Expose SSH port
EXPOSE 22

# Start SSH server
CMD ["/usr/sbin/sshd", "-D"]