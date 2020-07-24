FROM ubuntu:18.04
# FROM nvidia/cuda:8.0-cudnn6-runtime-ubuntu16.04
LABEL maintainer="yuki.oyabu@gmail.com"

COPY requirements.txt ./requirements.txt
ENV tz=Asia/Tokyo
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update \
    && apt-get -y upgrade \
    && apt-get install -y git \
    && apt-get install -y python3 \
    && apt-get install -y python3-tk \
    && apt-get install -y python3-pip \
    && apt-get install -y libsm6 libxext6 libxrender-dev \
    && apt-get install -y unzip wget \
    && pip3 install --upgrade pip \
    && pip3 install -r requirements.txt

