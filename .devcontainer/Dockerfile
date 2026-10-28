# SPDX-License-Identifier: GPL-3.0-or-later
# Development and CI environment for Vehkernel. Every tool that affects build
# output is pinned (ADR-0016); bump versions here and rebuild the container.
FROM ubuntu:24.04@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac

# The image supports linux/amd64 hosts only (ADR-0016): the Arm toolchain below
# is the x86_64 build. Stop here, before any download, on anything else.
RUN arch="$(dpkg --print-architecture)"; [ "$arch" = amd64 ] || { \
        echo "error: the Vehkernel development image supports linux/amd64 hosts only, not ${arch}." >&2; \
        echo "error: on an arm64 host, build it emulated (slow): docker build --platform linux/amd64 ..." >&2; \
        exit 1; }

ARG ARM_TOOLCHAIN_VERSION=14.2.rel1
ARG ARM_TOOLCHAIN_SHA256=62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823
ARG GTEST_VERSION=1.17.0
ARG GTEST_SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c

# OS packages come from the Ubuntu snapshot service at one timestamp, so every
# build installs the same versions; move the timestamp to update them. The
# snapshot is served over HTTPS only, so ca-certificates comes first from the
# release pocket, which never changes, and is then updated from the snapshot.
ARG UBUNTU_SNAPSHOT=20261027T000000Z
RUN mv /etc/apt/sources.list.d/ubuntu.sources /tmp/ \
    && echo "deb [signed-by=/usr/share/keyrings/ubuntu-archive-keyring.gpg] http://archive.ubuntu.com/ubuntu noble main" \
        > /etc/apt/sources.list.d/release.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm /etc/apt/sources.list.d/release.list \
    && mv /tmp/ubuntu.sources /etc/apt/sources.list.d/ \
    && echo "APT::Snapshot \"${UBUNTU_SNAPSHOT}\";" > /etc/apt/apt.conf.d/50vk-snapshot \
    && rm -rf /var/lib/apt/lists/*

# Host compiler for the sitl and test presets: GCC 13, named, not the g++
# metapackage, which also follows the release's default.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates curl git g++-13 python3 xz-utils \
    && rm -rf /var/lib/apt/lists/*
ENV CC=gcc-13 \
    CXX=g++-13

COPY --from=ghcr.io/astral-sh/uv:0.12.17@sha256:10787c682e4184e4f290de1171fd4703dc63de99221f10fe1c99002ce7fa9acc \
    /uv /uvx /usr/local/bin/

# Python tools (CMake, Ninja, clang-tidy, ...) from the repo's lock file.
ENV UV_PROJECT_ENVIRONMENT=/opt/vk-venv \
    UV_PYTHON_DOWNLOADS=never \
    UV_PYTHON=/usr/bin/python3
COPY pyproject.toml uv.lock .python-version /tmp/vk-tools/
RUN cd /tmp/vk-tools && uv sync --frozen && rm -rf /tmp/vk-tools
ENV PATH=/opt/vk-venv/bin:$PATH

# Arm GNU Toolchain for the STM32 targets.
RUN curl -fsSL -o /tmp/arm.tar.xz \
        "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi.tar.xz" \
    && echo "${ARM_TOOLCHAIN_SHA256}  /tmp/arm.tar.xz" | sha256sum -c - \
    && mkdir -p /opt/arm-gnu-toolchain \
    && tar -xJf /tmp/arm.tar.xz -C /opt/arm-gnu-toolchain --strip-components=1 \
    && rm /tmp/arm.tar.xz
ENV PATH=/opt/arm-gnu-toolchain/bin:$PATH

# GoogleTest, installed into the image so builds need no network.
RUN curl -fsSL -o /tmp/gtest.tar.gz \
        "https://github.com/google/googletest/archive/refs/tags/v${GTEST_VERSION}.tar.gz" \
    && echo "${GTEST_SHA256}  /tmp/gtest.tar.gz" | sha256sum -c - \
    && mkdir /tmp/gtest && tar -xzf /tmp/gtest.tar.gz -C /tmp/gtest --strip-components=1 \
    && cmake -S /tmp/gtest -B /tmp/gtest/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/googletest \
    && cmake --build /tmp/gtest/build && cmake --install /tmp/gtest/build \
    && rm -rf /tmp/gtest /tmp/gtest.tar.gz
ENV CMAKE_PREFIX_PATH=/opt/googletest

# Clang with libFuzzer and the sanitizer runtimes, for the fuzz preset only
# (ADR-0018); every other host build stays on GCC. llvm-18 brings
# llvm-symbolizer, so sanitizer reports name source lines. The snapshot pins
# their version; the explicit one also fails the build if a timestamp moves it.
ARG CLANG_VERSION=1:18.1.3-1ubuntu1
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        "clang-18=${CLANG_VERSION}" "libclang-rt-18-dev=${CLANG_VERSION}" "llvm-18=${CLANG_VERSION}" \
    && rm -rf /var/lib/apt/lists/*
ENV PATH=$PATH:/usr/lib/llvm-18/bin
