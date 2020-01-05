FROM nvidia/cuda:9.2-cudnn7-devel-ubuntu18.04

SHELL ["/bin/bash", "-c"]

# for ros2
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
        curl \
        gnupg2 \
        lsb-release \
        wget \
        libsm6 \
        libxext6 \
        libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl http://repo.ros2.org/repos.key | apt-key add -

RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update && apt-get install --no-install-recommends -y \
        git \
        python3-pip \
        python3-rospkg \
        python3-catkin-pkg \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

RUN pip3 install -U \
        argcomplete \
        setuptools

RUN apt-get update && apt-get install -y \
        ros-dashing-ros-base \
        ros-dashing-cv-bridge \
    && rm -rf /var/lib/apt/lists/*


RUN echo "source /opt/ros/dashing/setup.bash" >> /root/.bashrc

RUN apt-get update
RUN apt-get install -y --no-install-recommends \
                       apt-utils \
                       python3-dev \
                       python3-pip \
                       python3-setuptools \
                       python3-tk \
                       libopencv-dev

RUN pip3 install --upgrade pip

RUN pip3 install numpy==1.16.4 \
                 h5py==2.9.0 \
                 Cython==0.29.11 \
                 scipy==1.3.0 \
                 imageio==2.5.0 \
                 https://download.pytorch.org/whl/cu92/torch-0.4.1-cp36-cp36m-linux_x86_64.whl \
                 torchvision==0.2.0 \
                 opencv-python==4.1.0.25 \
                 cffi==1.12.3 \
                 sklearn \
                 numba==0.43 \
                 fire==0.1.3 \
                 openpyxl==2.6.2 \
                 motmetrics==1.1.3

# nvidia environment
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo 'export PATH=/usr/local/cuda/bin:${PATH}' >> ~/.bashrc
RUN echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}' >> ~/.bashrc

RUN apt-get update && \
    apt-get install -y --no-install-recommends mesa-utils x11-apps && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONDONTWRITEBYTECODE 1

# set default python vertion 3
RUN rm -f /usr/bin/python
RUN ln -s /usr/bin/python3 /usr/bin/python

WORKDIR /root

COPY ./ros_entrypoint.sh /
CMD ["/ros_entrypoint.sh"]
