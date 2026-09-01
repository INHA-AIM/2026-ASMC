# 팀 표준 ROS1 개발 컨테이너 (대회 규정: Ubuntu 20.04 + ROS Noetic)
FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

# Ubuntu mirror hash mismatch 완화 (Docker Desktop/WSL 간헐적)
RUN printf 'Acquire::Retries "5";\nAcquire::http::Pipeline-Depth "0";\n' \
    > /etc/apt/apt.conf.d/99docker-apt-fix

RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales && \
    ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 ko_KR.UTF-8 && \
    update-locale LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    git curl wget nano vim tmux sudo bash-completion \
    build-essential cmake libyaml-cpp-dev \
    python3-pip python3-dev \
    python3-scipy python3-numpy \
    iputils-ping net-tools mesa-utils \
    ros-noetic-rosbridge-server \
    ros-noetic-rosbridge-suite \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-cv-bridge \
    dmz-cursor-theme xcursors-all \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-lbc.txt requirements-mgeo.txt requirements-grpc.txt /tmp/
RUN pip3 install --no-cache-dir --upgrade "pip<24.1"

# ROS 컨테이너에서도 신호등 추론이 RTX 40(Ada) GPU를 사용하도록 CUDA wheel을
# 명시한다. ultralytics보다 먼저 설치해 CPU용 torch가 간접 설치되지 않게 한다.
RUN pip3 install --no-cache-dir \
      torch==2.1.2 \
      torchvision==0.16.2 \
      --index-url https://download.pytorch.org/whl/cu118

RUN pip3 install --no-cache-dir \
      -r /tmp/requirements-lbc.txt \
      -r /tmp/requirements-mgeo.txt \
      -r /tmp/requirements-grpc.txt \
      tqdm ultralytics && \
    rm -f /tmp/requirements-*.txt && \
    python3 -c "from scipy.spatial import cKDTree; import cv2; import numpy"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ws/devel/setup.bash ] && source /root/ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export LANG=en_US.UTF-8" >> /root/.bashrc && \
    echo "export LC_ALL=en_US.UTF-8" >> /root/.bashrc

WORKDIR /root/ws

ENV QT_X11_NO_MITSHM=1
ENV XCURSOR_PATH=/usr/share/icons
ENV XCURSOR_THEME=DMZ-White

CMD ["bash"]
