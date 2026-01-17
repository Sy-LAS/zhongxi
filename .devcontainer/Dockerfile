# syntax=docker/dockerfile:1.4
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# 设置代理 (如果需要)
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG NO_PROXY
ARG APT_PROXY

ENV HTTP_PROXY=${HTTP_PROXY:-} \
    HTTPS_PROXY=${HTTPS_PROXY:-} \
    NO_PROXY=${NO_PROXY:-} \
    http_proxy=${HTTP_PROXY:-} \
    https_proxy=${HTTPS_PROXY:-} \
    no_proxy=${NO_PROXY:-}

# 设置 apt 代理
RUN set -eux; \
    if [ -n "${APT_PROXY:-}" ]; then \
      printf 'Acquire::http::Proxy "%s";\nAcquire::https::Proxy "%s";\n' "${APT_PROXY}" "${APT_PROXY}" >/etc/apt/apt.conf.d/99-proxy; \
    fi; \
    apt-get update; \
    rosdep update; \
    apt-get install -y --no-install-recommends \
      ca-certificates gnupg2 gpg-agent curl wget lsb-release locales \
      python3-pip python3-colcon-common-extensions \
      build-essential cmake git vim nano less \
      libx11-6 libxext6 libxrender1 libxkbcommon0 libxkbcommon-x11-0 libxi6 libxrandr2 \
      libxcb1 libxcb-render0 libxcb-shape0 libxcb-xfixes0 \
      mesa-utils libgl1 libglu1-mesa \
      x11-apps \
      ros-humble-rviz2 ros-humble-rqt ros-humble-rqt-common-plugins \
      ros-humble-ros-gz ros-humble-ros-gz-sim \
      sudo \
      libwayland-dev \
      wayland-protocols \
    ; \
    locale-gen C.UTF-8 || true; \
    update-locale LANG=C.UTF-8 LC_ALL=C.UTF-8 || true; \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/*

# 设置 ROS 环境变量
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 设置 XDG_RUNTIME_DIR 环境变量
RUN echo "export XDG_RUNTIME_DIR=/tmp/vscode-wayland-runtime" >> /root/.bashrc

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
