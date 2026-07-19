FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Root: install system packages
RUN apt-get update && apt-get install -y \
    git python3 python3-pip sudo wget curl \
    build-essential gcc g++ cmake \
    tzdata lsb-release gnupg \
    && rm -rf /var/lib/apt/lists/*

# Root: python packages sim_vehicle.py needs
RUN pip3 install pymavlink MAVProxy pexpect matplotlib

# install-prereqs-ubuntu.sh refuses to run as root — create a build user
RUN useradd -m -s /bin/bash ardupilot && \
    echo "ardupilot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ardupilot
ENV USER=ardupilot
WORKDIR /home/ardupilot

# Clone ArduPilot stable branch (master was causing SIGFPE in SITL due to
# feenableexcept + unguarded 1/airspeed in QuadPlane init).
RUN git clone --depth 1 --branch ArduPlane-stable \
        https://github.com/ArduPilot/ardupilot.git ardupilot && \
    cd ardupilot && \
    git submodule update --init --recursive --depth 1

WORKDIR /home/ardupilot/ardupilot

# Install ArduPilot build prerequisites (installs additional apt packages via sudo)
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Configure waf (no compile yet — patches applied first)
RUN ./waf configure --board sitl

# Apply source patches before compiling (fixes Docker UDP one-way NAT and SITL panics)
COPY --chown=ardupilot:ardupilot patches/apply_patches.py /home/ardupilot/apply_patches.py
RUN python3 /home/ardupilot/apply_patches.py

# Build ArduPlane (VTOL) and ArduCopter (drone) SITL binaries with patches
# applied. entrypoint.sh selects the binary via the AP_BINARY env var.
RUN ./waf plane
RUN ./waf copter

# Copy param file and entrypoint
COPY --chown=ardupilot:ardupilot vtol.parm /home/ardupilot/ardupilot/vtol.parm
COPY --chown=ardupilot:ardupilot entrypoint.sh /home/ardupilot/entrypoint.sh
RUN chmod +x /home/ardupilot/entrypoint.sh

# SITL writes runtime files (eeprom.bin, logs) here
RUN mkdir -p /home/ardupilot/sitl
WORKDIR /home/ardupilot/sitl

EXPOSE 9002/udp
EXPOSE 14560/udp

ENTRYPOINT ["/home/ardupilot/entrypoint.sh"]
