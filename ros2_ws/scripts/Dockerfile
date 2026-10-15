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

# 使用 rosdepc 自動解析並安裝源碼所需的 ROS 依賴
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple rosdepc --break-system-packages && \
    rosdepc init && \
    rosdepc update --rosdistro jazzy && \
    rosdepc install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-c"]
