FROM nvidia/cuda:9.0-cudnn7-devel-ubuntu16.04

ENV DEBIAN_FRONTEND=noninteractive

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
                 torch==0.3.1 \
                 torchvision==0.2.0 \
                 opencv-python==4.1.0.25 \
                 cffi==1.12.3 \
                 sklearn==0.19 \
                 numba==0.43 \
                 fire==0.1.3 \
                 openpyxl==2.6.2 \
                 motmetrics == 1.1.3

# nvidia environment
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && \
    apt-get install -y --no-install-recommends mesa-utils x11-apps && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONDONTWRITEBYTECODE 1

WORKDIR /root