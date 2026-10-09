FROM ubuntu:24.04

ARG ARDUPILOT_REF=Copter-4.6.2
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git ca-certificates python3 python3-dev python3-pip python3-venv \
    build-essential ccache gawk wget pkg-config \
    libtool libxml2-dev libxslt1-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ardupilot
RUN git clone --depth 1 --branch ${ARDUPILOT_REF} https://github.com/ArduPilot/ardupilot.git . \
    && git submodule update --init --recursive

RUN useradd --create-home --shell /bin/bash ardupilot \
    && chown -R ardupilot:ardupilot /ardupilot
USER ardupilot
ENV USER=ardupilot

RUN python3 -m pip install --user --break-system-packages \
    "pymavlink>=2.4.40" MAVProxy future lxml pyserial \
    "empy==3.3.4" pyyaml numpy pexpect

# The JSON backend receives one ideal Isaac IMU.  Keep exactly one IMU enabled
# and one scheduler rate: duplicated parameters make the effective setting
# dependent on parser ordering and were the source of the gyro-rate pre-arm
# failure on slower Isaac render loops.
RUN sed -i \
    -e 's/^INS_ACC2OFFS_X.*/INS_ACC2OFFS_X 0/' \
    -e 's/^INS_ACC2OFFS_Y.*/INS_ACC2OFFS_Y 0/' \
    -e 's/^INS_ACC2OFFS_Z.*/INS_ACC2OFFS_Z 0/' \
    -e 's/^INS_ACC2SCAL_X.*/INS_ACC2SCAL_X 1/' \
    -e 's/^INS_ACC2SCAL_Y.*/INS_ACC2SCAL_Y 1/' \
    -e 's/^INS_ACC2SCAL_Z.*/INS_ACC2SCAL_Z 1/' \
    -e '/^INS_ENABLE_MASK /d' \
    -e '/^INS_USE2 /d' \
    -e '/^INS_USE3 /d' \
    -e '/^SCHED_LOOP_RATE /d' \
    -e '/^SIM_RATE_HZ /d' \
    -e '/^FRAME_CLASS /d' \
    -e '/^FRAME_TYPE /d' \
    -e '/^ATC_RAT_YAW_P /d' \
    -e '/^ATC_RAT_YAW_I /d' \
    /ardupilot/Tools/autotest/default_params/copter.parm \
    && printf '\nINS_ENABLE_MASK 1\nINS_USE2 0\nINS_USE3 0\nSCHED_LOOP_RATE 400\nSIM_RATE_HZ 800\nFRAME_CLASS 1\nFRAME_TYPE 1\nATC_RAT_YAW_P 0.04\nATC_RAT_YAW_I 0.004\n' >> /ardupilot/Tools/autotest/default_params/copter.parm

RUN mkdir -p /ardupilot/logs && chown ardupilot:ardupilot /ardupilot/logs

RUN ./waf configure --board sitl && ./waf copter
