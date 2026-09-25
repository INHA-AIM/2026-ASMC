# 팀 ML 모델 학습·validation 컨테이너
# 실시간 또는 bag replay ROS 추론은 asmc-ros-noetic에서 실행한다.
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip python3-dev python3-venv \
    git wget curl nano vim \
    build-essential cmake \
    libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev \
    software-properties-common lsb-release gnupg2 \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    ros-noetic-sensor-msgs \
    ros-noetic-cv-bridge \
    python3-rospkg python3-catkin-pkg python3-empy \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir --ignore-installed PyYAML==6.0.1

COPY requirements-train.txt /tmp/
RUN python3 -m pip install --no-cache-dir --retries 10 --timeout 120 \
    -r /tmp/requirements-train.txt && \
    rm /tmp/requirements-train.txt

RUN python3 -m pip install --no-cache-dir --retries 20 --timeout 300 \
    torch==2.1.2+cu118 \
    torchvision==0.16.2+cu118 \
    torchaudio==2.1.2+cu118 \
    --index-url https://download.pytorch.org/whl/cu118

WORKDIR /root/ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
