FROM archlinux

RUN pacman -Syu --needed --noconfirm base-devel git go nvidia
RUN git clone https://aur.archlinux.org/yay.git
RUN sudo chmod 777 -R ./yay

RUN useradd --shell=/bin/bash build && usermod -L build
RUN echo "build ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN echo "root ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN mkdir /home/build
RUN chmod -R 777 /home/build && chown -R build:build /home/build

USER build
RUN cd yay && makepkg -si --noconfirm
RUN yay -Syu --noconfirm bullet-git gazebo eigen-git
RUN rm -rf /home/build/.cache/yay/*
USER root

COPY . /src
COPY . /root/.gazebo/models/simplified_biped

WORKDIR /src/control_plugin

RUN rm -rf ./build && mkdir ./build
WORKDIR /src/control_plugin/build

RUN cmake ..
RUN make -j

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,video,utility

CMD bash -c 'gazebo --verbose ../../simplified_biped.world'
# Current command: sudo docker run -it --gpus all --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" loukordos/biped-sim