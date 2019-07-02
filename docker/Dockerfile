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

RUN pip3 install numpy \
                 h5py \
                 Cython \
                 scipy \
                 imageio \
                 torch==0.3.1 \
                 torchvision==0.2.0 \
                 opencv-python \
                 cffi \
                 sklearn \
                 numba \
                 fire \
                 motmetrics

# nvidia environment
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && \
    apt-get install -y --no-install-recommends mesa-utils x11-apps && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONDONTWRITEBYTECODE 1

WORKDIR /root