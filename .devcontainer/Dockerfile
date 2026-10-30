# SPDX-License-Identifier: GPL-3.0-or-later
# Development and CI environment for Vehkernel. Every tool that affects build
# output is pinned (ADR-0016); bump versions here and rebuild the container.
FROM ubuntu:24.04@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac

# The image supports linux/amd64 and linux/arm64 hosts (ADR-0016). Stop here,
# before any download, on anything else. BuildKit sets TARGETARCH; falling back
# to dpkg --print-architecture handles plain docker build without BuildKit.
ARG TARGETARCH
RUN arch="${TARGETARCH:-$(dpkg --print-architecture)}"; [ "$arch" = amd64 ] || [ "$arch" = arm64 ] || { \
        echo "error: the Vehkernel development image supports linux/amd64 and linux/arm64 hosts only, not ${arch}." >&2; \
        exit 1; }

ARG ARM_TOOLCHAIN_VERSION=14.2.rel1
ARG ARM_TOOLCHAIN_X86_64_SHA256=62a63b981fe391a9cbad7ef51b17e49aeaa3e7b0d029b36ca1e9c3b2a9b78823
ARG ARM_TOOLCHAIN_AARCH64_SHA256=87330bab085dd8749d4ed0ad633674b9dc48b237b61069e3b481abd364d0a684
ARG UV_VERSION=0.12.17
ARG UV_X86_64_SHA256=fa82fd8dde8e8eefdecada6aa0889666556cfceb690d06e0c3bca49eb3070a63
ARG UV_AARCH64_SHA256=d636d1b678e9e7f367ecb22b46bd1cabbed234d6bc3b4d96365d2b507f72f86c
ARG GTEST_VERSION=1.17.0
ARG GTEST_SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c

# OS packages come from the Ubuntu snapshot service at one timestamp, so every
# build installs the same versions; move the timestamp to update them. The
# snapshot is served over HTTPS only, so ca-certificates comes first from the
# release pocket, which never changes, and is then updated from the snapshot.
# apt maps archive.ubuntu.com and security.ubuntu.com to the snapshot service
# but not ports.ubuntu.com, where arm64 comes from, so that mapping is added.
ARG UBUNTU_SNAPSHOT=20261027T000000Z
RUN arch="${TARGETARCH:-$(dpkg --print-architecture)}" \
    && case "$arch" in \
        amd64) mirror="http://archive.ubuntu.com/ubuntu" ;; \
        arm64) mirror="http://ports.ubuntu.com/ubuntu-ports" ;; \
        *) exit 1 ;; \
    esac \
    && mv /etc/apt/sources.list.d/ubuntu.sources /tmp/ \
    && echo "deb [signed-by=/usr/share/keyrings/ubuntu-archive-keyring.gpg] ${mirror} noble main" \
        > /etc/apt/sources.list.d/release.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm /etc/apt/sources.list.d/release.list \
    && mv /tmp/ubuntu.sources /etc/apt/sources.list.d/ \
    && { echo "APT::Snapshot \"${UBUNTU_SNAPSHOT}\";"; \
         echo 'Acquire::Snapshots::URI::Host::ports.ubuntu.com "https://snapshot.ubuntu.com/@PATH@/@SNAPSHOTID@/";'; \
       } > /etc/apt/apt.conf.d/50vk-snapshot \
    && rm -rf /var/lib/apt/lists/*

# Host compiler for the sitl and test presets: GCC 13, named, not the g++
# metapackage, which also follows the release's default.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates curl git g++-13 python3 xz-utils \
    && rm -rf /var/lib/apt/lists/*
ENV CC=gcc-13 \
    CXX=g++-13

# uv is installed from its release tarball rather than its multi-arch image, so
# that an unsupported architecture reaches the stop step instead of failing on
# BuildKit's image resolution.
RUN arch="${TARGETARCH:-$(dpkg --print-architecture)}" \
    && case "$arch" in \
        amd64) uv_arch=x86_64; uv_sha256="${UV_X86_64_SHA256}" ;; \
        arm64) uv_arch=aarch64; uv_sha256="${UV_AARCH64_SHA256}" ;; \
        *) exit 1 ;; \
    esac \
    && curl -fsSL -o /tmp/uv.tar.gz \
        "https://github.com/astral-sh/uv/releases/download/${UV_VERSION}/uv-${uv_arch}-unknown-linux-gnu.tar.gz" \
    && echo "${uv_sha256}  /tmp/uv.tar.gz" | sha256sum -c - \
    && tar -xzf /tmp/uv.tar.gz -C /usr/local/bin --strip-components=1 \
    && rm /tmp/uv.tar.gz

# Python tools (CMake, Ninja, clang-tidy, ...) from the repo's lock file.
ENV UV_PROJECT_ENVIRONMENT=/opt/vk-venv \
    UV_PYTHON_DOWNLOADS=never \
    UV_PYTHON=/usr/bin/python3
COPY pyproject.toml uv.lock .python-version /tmp/vk-tools/
RUN cd /tmp/vk-tools && uv sync --frozen && rm -rf /tmp/vk-tools
ENV PATH=/opt/vk-venv/bin:$PATH

# Arm GNU Toolchain for the STM32 targets.
RUN arch="${TARGETARCH:-$(dpkg --print-architecture)}" \
    && case "$arch" in \
        amd64) toolchain_arch=x86_64; toolchain_sha256="${ARM_TOOLCHAIN_X86_64_SHA256}" ;; \
        arm64) toolchain_arch=aarch64; toolchain_sha256="${ARM_TOOLCHAIN_AARCH64_SHA256}" ;; \
        *) exit 1 ;; \
    esac \
    && curl -fsSL -o /tmp/arm.tar.xz \
        "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-${toolchain_arch}-arm-none-eabi.tar.xz" \
    && echo "${toolchain_sha256}  /tmp/arm.tar.xz" | sha256sum -c - \
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
