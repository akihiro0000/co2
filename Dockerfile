FROM debian:buster

RUN apt update && apt upgrade -y
RUN apt install -y curl python3 python3-pip python3-dev python3 -V git wget vim

RUN pip3 install RPi.GPIO paho-mqtt datetime


WORKDIR /root
