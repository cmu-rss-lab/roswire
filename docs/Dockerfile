FROM alpine:3.7
RUN apk add --no-cache \
      openssl-dev \
      git \
      python3 \
      python3-dev \
      py3-pip \
      build-base \
      docker \
      gcc \
      linux-headers \
      openjdk8 \
      libffi-dev \
 && rm -rf /tmp/*
COPY ./requirements.dev.txt /tmp
RUN pip3 install -r /tmp/requirements.dev.txt \
  && rm /tmp/*
COPY . /opt/roswire
RUN pip3 install -e /opt/roswire
