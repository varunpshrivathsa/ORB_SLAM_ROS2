FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    unzip \
    pkg-config \
    python3 \
    python3-pip \
    libeigen3-dev \
    libopencv-dev \
    libglew-dev \
    libboost-all-dev \
    libgtk-3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
