# ROS 2 ARM64 Cross-compilation Target Sysroot Environment
FROM ros:jazzy

WORKDIR /ros_test_ws

# 換源加速 (可根據需求替換為其他 apt 鏡像源)
RUN sed -i 's/ports.ubuntu.com/mirrors.tuna.tsinghua.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources

# 安裝構建底層依賴與傳感器/底盤驅動所需庫
RUN apt-get update && apt-get install -y \
   libssl-dev \
   libopenni2-dev \
   libusb-1.0-0-dev \
   libuvc-dev \
   libgoogle-glog-dev \
   nlohmann-json3-dev

# 拷貝源碼目錄 (相對於 Docker build context，即 scripts 上一層的 src)
COPY ./src /ros_test_ws/src

# 使用官方 rosdep 自動解析並安裝源碼所需的 ROS 依賴
# 抑制 Docker 容器內以 root 執行 rosdep update 時的常規權限提醒
RUN sed -i "s/if os.geteuid() == 0:/if False:/g" /usr/lib/python3/dist-packages/rosdep2/main.py && \
    apt-get update && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-c"]
