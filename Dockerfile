FROM alpine

RUN echo "http://dl-cdn.alpinelinux.org/alpine/v3.5/community" >> /etc/apk/repositories && apk update

RUN apk add --update-cache --no-cache build-base pkgconfig git libgcc cmake

# Install python/pip
ENV PYTHONUNBUFFERED=1
RUN apk add --update --no-cache python3 && ln -sf python3 /usr/bin/python
RUN python3 -m ensurepip
RUN pip3 install --no-cache --upgrade pip setuptools

# Install Ignition CMake 2 (Docs do not specify desired version for gazebo 11 so latest is used)

RUN git clone https://github.com/ignitionrobotics/ign-cmake /tmp/ign-cmake
WORKDIR /tmp/ign-cmake
# Latest ign-cmake-2 commit hash at time of writing Dockerfile
RUN git checkout 3b7870289027ae0fc04c83d5817907ed863a0720
RUN mkdir build
WORKDIR /tmp/ign-cmake/build
RUN cmake ../
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install

# Gazebo 11 needs Igntion Math 6

RUN git clone https://github.com/ignitionrobotics/ign-math /tmp/ign-math
WORKDIR /tmp/ign-math
# Latest ign-math-6 commit hash at time of writing Dockerfile
RUN git checkout 9519ce74dae8b8c68303ef81e85fda27d9f4f494
RUN mkdir build
WORKDIR /tmp/ign-math/build
RUN cmake ../
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install

# Gazebo 11 needs Ignition Utils 1

RUN git clone https://github.com/ignitionrobotics/ign-utils.git /tmp/ign-utils
WORKDIR /tmp/ign-utils
# Latest ign-utils-1 commit hash at time of writing Dockerfile
RUN git checkout 8222e994f11b118c08afac61a6b0a3084fb08032
RUN mkdir build
WORKDIR /tmp/ign-utils/build
RUN cmake ../
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install
# # Deps
# RUN pip3 install vcstool colcon-common-extensions


# Gazebo 11 needs Ignition Common 3

# Deps
RUN apk add --update-cache --no-cache freeimage-dev tinyxml2-dev ffmpeg-dev protoc protobuf-dev glib-dev
# libgts
RUN git clone https://github.com/LouKordos/libgts.git /tmp/libgts
WORKDIR /tmp/libgts
RUN ./configure
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install

RUN git clone https://github.com/ignitionrobotics/ign-common /tmp/ign-common
WORKDIR /tmp/ign-common
# Latest ign-common-3 commit hash at time of writing Dockerfile
RUN git checkout ee5ba8e596e86e41dcec61847c3b6bdf0fa8c04a
RUN mkdir build
WORKDIR /tmp/ign-common/build
# Patch include error for file descriptor calls due to compiling on alpine
RUN sed -i '1i #include <sys/select.h>' /tmp/ign-common/profiler/src/Remotery/lib/Remotery.c
RUN cmake  ../
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install

# SDF9 needs ign-tools-1

# Deps
RUN apk add --update-cache --no-cache ruby-dev

RUN git clone https://github.com/ignitionrobotics/ign-tools.git /tmp/ign-tools
WORKDIR /tmp/ign-tools
# Latest ign-tools-1 commit hash at time of writing Dockerfile
RUN git checkout 2cca470de7a62327beeb1065209c8ffd7a303d29
RUN mkdir build
WORKDIR /tmp/ign-tools/build
RUN cmake ../
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install

# Gazebo 11 needs SDF 9

# Deps
RUN apk add --update-cache --no-cache boost-dev tinyxml-dev libxml2-utils ruby-dev ruby
RUN git clone https://github.com/osrf/sdformat /tmp/sdformat
WORKDIR /tmp/sdformat
# Latest sdf9 commit hash at time of writing Dockerfile
RUN git checkout 9876a6089c88ac39408e658b6b4b2928abab8377
RUN mkdir build
WORKDIR /tmp/sdformat/build
RUN cmake ../
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install

# Gazebo 11 needs Ingition Messages 5
RUN git clone https://github.com/ignitionrobotics/ign-msgs /tmp/ign-msgs
WORKDIR /tmp/ign-msgs
# Latest ign-msgs-5 commit hash at time of writing Dockerfile
RUN git checkout c7b64782a0b046ec108d2e2cb2cb0ee13e8dff21

#ifdef major
#undef major
#endif
#ifdef minor
#undef minor
#endif

RUN mkdir build
WORKDIR /tmp/ign-msgs/build
RUN cmake ../
RUN make -j$(awk '( $1 == "MemTotal:" ) { printf "%d", $2/2/1000000 }' /proc/meminfo)
RUN make install

# Gazebo 11 needs Ignition Fuel Tools 4

# Deps
RUN apk add --update-cache --no-cache zip

# Gazebo 11 needs Ignition Transport 8


# Clear dependency source files
RUN rm -rf /tmp/*

COPY . /src
COPY . /root/.gazebo/models/simplified_biped

WORKDIR /src/control_plugin

RUN rm -rf ./build && mkdir ./build
WORKDIR /src/control_plugin/build

RUN cmake ..
RUN make -j$(nproc)

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,video,utility

CMD bash -c 'gazebo --verbose ../../simplified_biped.world'
# Current command: sudo docker run -it --gpus all --net=host -e "TERM=xterm-256color" --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" loukordos/biped-sim