# syntax=docker/dockerfile:1.4
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

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

# 切换镜像源（优先清华，失败回退官方）
RUN set -eux; \
    . /etc/os-release; \
    tsinghua_setup() { \
      if [ "${ID}" = "ubuntu" ] && [ "${VERSION_CODENAME}" = "jammy" ]; then \
        sed -i 's|http://archive.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' /etc/apt/sources.list; \
        sed -i 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list; \
      fi; \
      if [ -f /etc/apt/sources.list.d/ros2.list ]; then \
        sed -i 's|http://packages.ros.org/ros2|https://mirrors.tuna.tsinghua.edu.cn/ros2|g' /etc/apt/sources.list.d/ros2.list; \
      fi; \
    }; \
    official_setup() { \
      if [ "${ID}" = "ubuntu" ] && [ "${VERSION_CODENAME}" = "jammy" ]; then \
        sed -i 's|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|http://archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list; \
        sed -i 's|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|http://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list; \
      fi; \
      if [ -f /etc/apt/sources.list.d/ros2.list ]; then \
        sed -i 's|https://mirrors.tuna.tsinghua.edu.cn/ros2|http://packages.ros.org/ros2|g' /etc/apt/sources.list.d/ros2.list; \
      fi; \
    }; \
    if [ -n "${APT_PROXY:-}" ]; then \
      printf 'Acquire::http::Proxy "%s";\nAcquire::https::Proxy "%s";\n' "${APT_PROXY}" "${APT_PROXY}" >/etc/apt/apt.conf.d/99-proxy; \
    fi; \
    tsinghua_setup; \
    if ! apt-get -o Acquire::Retries=3 update; then \
      echo "TUNA mirror failed, falling back to official..."; \
      official_setup; \
      apt-get -o Acquire::Retries=3 update; \
    fi; \
    # Wayland/Qt/GL/ROS GUI 依赖（均为 jammy 有效包名）
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      ca-certificates gnupg2 gpg-agent curl wget lsb-release locales \
      python3-pip python3-colcon-common-extensions \
      build-essential cmake git vim nano less sudo \
      # XKB/输入
      libxkbcommon0 libxkbcommon-x11-0 \
      # Wayland & EGL/GBM
      libwayland-client0 libwayland-cursor0 libwayland-egl1 libegl1 libgbm1 \
      # OpenGL
      mesa-utils libgl1 libglu1-mesa \
      # Qt5 + Wayland 平台支持（关键）
      libqt5gui5 qtwayland5 libqt5waylandclient5 \
      # 可选窗口装饰
      libdecor-0-0 \
      # ROS GUI 应用
      ros-humble-rviz2 ros-humble-rqt ros-humble-rqt-common-plugins \
      # Gazebo/桥接（可按需移除）
      ros-humble-ros-gz ros-humble-ros-gz-sim \
    ; \
    locale-gen C.UTF-8 || true; \
    update-locale LANG=C.UTF-8 LC_ALL=C.UTF-8 || true; \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/*

# Wayland 运行时目录（root 的 XDG_RUNTIME_DIR），权限必须 0700
RUN mkdir -p /run/user/0 && chmod 700 /run/user/0

# ROS 环境 + 默认让 rviz2 用 X11 + 软件渲染（可用为先）
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo 'export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}' >> /root/.bashrc && \
    echo 'export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}' >> /root/.bashrc

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]
CMD ["bash"]
