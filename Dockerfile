FROM ubuntu:20.04

RUN apt-get update && apt-get install -y
RUN apt-get install -y apt-utils && apt-get install -y wget

RUN apt-get install -y build-essential
RUN apt-get install -y g++
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ xenial main universe" >> /etc/apt/sources.list
RUN echo "deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main universe" >> /etc/apt/sources.list
RUN apt-get update && apt-get install -y

WORKDIR /TSP/

RUN rm -rf *


COPY Makefile ./

RUN make dependencies

VOLUME TSP/

ENTRYPOINT ["/usr/bin/make"]

