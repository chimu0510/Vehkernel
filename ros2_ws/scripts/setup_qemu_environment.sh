#!/bin/bash
# Setup QEMU environment, build target ARM64 sysroot via Docker, and export it for cross-compilation

set -e

ARCH="arm64"
TARGET_ARCH="linux/${ARCH}"
TAG="ros2_cross_compile:jazzy-${ARCH}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${WORKSPACE_DIR}/src"
SYSROOT_DIR="${WORKSPACE_DIR}/ubuntu_${ARCH}"

echo -e "\033[1;32m=== ROS 2 ARM64 Cross-compilation Environment Setup ===\033[0m"
echo -e "Script Directory:    ${SCRIPT_DIR}"
echo -e "Workspace Directory: ${WORKSPACE_DIR}"
echo -e "Target Source Dir:   ${SRC_DIR}"
echo -e "Target Sysroot Dir:  ${SYSROOT_DIR}"

# Gain root access
if [ "$EUID" -ne 0 ]; then
    echo -e "\033[1;33mChecking sudo permissions...\033[0m"
    sudo true
fi

# Check Docker
if ! command -v docker >/dev/null 2>&1; then
    echo -e "\033[1;31mError: Docker is not installed. Please install Docker first.\033[0m"
    exit 1
fi

# Ensure src directory exists for Docker build context
if [ ! -d "${SRC_DIR}" ]; then
    echo -e "\033[1;33mTarget source directory ${SRC_DIR} does not exist. Creating empty src directory...\033[0m"
    mkdir -p "${SRC_DIR}"
fi

# Register QEMU binfmt handlers BEFORE building (RUN steps in an arm64 image need emulation)
echo -e "\033[1;32mChecking QEMU binfmt emulation for ${ARCH}...\033[0m"
if [ ! -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
    echo -e "\033[1;33mQEMU binfmt handler for aarch64 not registered. Registering...\033[0m"
    docker run --privileged --rm tonistiigi/binfmt --install "${ARCH}"
fi
if ! docker run --rm --platform="${TARGET_ARCH}" ros:jazzy uname -m 2>/dev/null | grep -q aarch64; then
    echo -e "\033[1;31mError: Failed to run ARM64 container under QEMU.\033[0m"
    exit 1
fi

# Build Docker image using the workspace root as build context
echo -e "\033[1;32mBuilding Docker image (${TAG}) for ${TARGET_ARCH}...\033[0m"
docker build -f "${SCRIPT_DIR}/Dockerfile" --platform="${TARGET_ARCH}" -t "${TAG}" --load "${WORKSPACE_DIR}"

# Export Docker container filesystem as Sysroot
echo -e "\033[1;32mExporting target sysroot filesystem to ${SYSROOT_DIR}...\033[0m"
CONTAINER_ID=$(docker create "${TAG}")
if [ -z "${CONTAINER_ID}" ]; then
    echo -e "\033[1;31mError: Failed to create temporary container from ${TAG}.\033[0m"
    exit 1
fi

if [ -d "${SYSROOT_DIR}" ]; then
    echo -e "Removing existing sysroot directory..."
    sudo rm -rf "${SYSROOT_DIR}"
fi

sudo mkdir -p "${SYSROOT_DIR}"
echo -e "Extracting container rootfs (this may take a few minutes)..."
sudo docker export "${CONTAINER_ID}" | sudo tar -xC "${SYSROOT_DIR}"
sudo docker rm "${CONTAINER_ID}"

# Prevent colcon from scanning the sysroot directory
echo -e "Adding COLCON_IGNORE to sysroot..."
sudo touch "${SYSROOT_DIR}/COLCON_IGNORE"

# Prepare workspace mount / symlink target in sysroot
sudo mkdir -p "${SYSROOT_DIR}/workspace"
sudo chown -R "${USER}:${USER}" "${SYSROOT_DIR}/workspace"

# Install host cross-compilers
echo -e "\033[1;32mInstalling GNU cross-compilers on host system...\033[0m"
sudo apt-get update && sudo apt-get install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Fix sysroot paths for CMake
echo -e "\033[1;32mNormalizing CMake paths inside sysroot...\033[0m"
if [ -f "${SCRIPT_DIR}/fix_sysroot_paths.sh" ]; then
    bash "${SCRIPT_DIR}/fix_sysroot_paths.sh" "${SYSROOT_DIR}"
fi

echo -e "\033[1;32m=== Setup Complete! You can now run ./scripts/cross_compile.sh ===\033[0m"
