#!/bin/bash
# Cross-compilation and deployment script for ROS 2 ARM64 (aarch64)
# Dynamically resolves all paths relative to the script location (no hardcoded absolute paths)

set -e

# Help / Usage info
if [ "$1" == "help" ] || [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo -e "\033[1;32mROS 2 Cross Compile for ARM64 (aarch64)\033[0m"
    echo -e "\033[1;33mUsage:\033[0m"
    echo -e "  $0                                   # Build ROS 2 packages in src/"
    echo -e "  $0 pack [target_path]                # Package complete workspace (install + build + log + src) into ugv_ws.tar.gz"
    echo -e "  $0 dest <port> <user>@<ip>:<path>    # Build and deploy complete workspace (install + build + log + src) to target"
    echo -e "\033[1;33mExamples:\033[0m"
    echo -e "  $0 pack /home/ubuntu/ugv_ws"
    echo -e "  $0 dest 22 ubuntu@192.168.1.100:/home/ubuntu/ugv_ws\033[0m"
    exit 0
fi

# Dynamically determine paths relative to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${WORKSPACE_DIR}/src"

echo -e "\033[1;32m=== Starting ROS 2 Cross Compilation ===\033[0m"
echo -e "Script Directory:    ${SCRIPT_DIR}"
echo -e "Workspace Directory: ${WORKSPACE_DIR}"
echo -e "Target Source Dir:   ${SRC_DIR}"

# Validate source directory
if [ ! -d "${SRC_DIR}" ]; then
    echo -e "\033[1;31mError: Target source directory not found at '${SRC_DIR}'\033[0m"
    echo -e "\033[1;33mPlease create '${SRC_DIR}' and place the ROS 2 packages to be compiled inside.\033[0m"
    exit 1
fi

# Check for packages in src/
if [ -z "$(find "${SRC_DIR}" -mindepth 1 -maxdepth 2 -name "package.xml" 2>/dev/null)" ]; then
    echo -e "\033[1;33mWarning: No 'package.xml' found directly in '${SRC_DIR}'. Ensure your packages are located here.\033[0m"
fi

# Locate Target Sysroot
if [ -z "${SYSROOT}" ]; then
    if [ -d "${WORKSPACE_DIR}/ubuntu_arm64" ]; then
        export SYSROOT="${WORKSPACE_DIR}/ubuntu_arm64"
    elif [ -d "${SCRIPT_DIR}/ubuntu_arm64" ]; then
        export SYSROOT="${SCRIPT_DIR}/ubuntu_arm64"
    else
        echo -e "\033[1;31mError: Target sysroot directory 'ubuntu_arm64' not found.\033[0m"
        echo -e "\033[1;33mPlease run '${SCRIPT_DIR}/setup_qemu_environment.sh' first, or export SYSROOT=/path/to/sysroot\033[0m"
        exit 1
    fi
fi

# Target Architecture and ROS Environment Configuration
export TARGET_ARCH=aarch64
export PYTHON_SOABI="${PYTHON_SOABI:-cpython-312-aarch64-linux-gnu}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS2_INSTALL_PATH="${SYSROOT}/opt/ros/${ROS_DISTRO}"
export ROS_WS_INSTALL_PATH="/workspace/install"

echo -e "Target Architecture: ${TARGET_ARCH}"
echo -e "Target SYSROOT:      ${SYSROOT}"
echo -e "ROS 2 Distro:        ${ROS_DISTRO}"

if [ ! -f "${ROS2_INSTALL_PATH}/setup.bash" ]; then
    echo -e "\033[1;31mError: ROS 2 setup.bash not found at '${ROS2_INSTALL_PATH}/setup.bash'\033[0m"
    exit 1
fi

# Source target ROS 2 environment from Sysroot
source "${ROS2_INSTALL_PATH}/setup.bash"

# Link local workspace install directory into Sysroot workspace for package chaining
mkdir -p "${SYSROOT}/workspace"
rm -rf "${SYSROOT}${ROS_WS_INSTALL_PATH}"
mkdir -p "${WORKSPACE_DIR}/install"
ln -s "${WORKSPACE_DIR}/install" "${SYSROOT}${ROS_WS_INSTALL_PATH}"

# Switch to workspace directory for standard colcon build execution
cd "${WORKSPACE_DIR}"

PYTHON_HOST_BIN="$(command -v python3)"

echo -e "\033[1;32mInvoking colcon build on target directory: src ...\033[0m"

if ! colcon build \
    --base-paths src \
    --merge-install \
    --cmake-force-configure \
    --cmake-args \
        -Wno-dev \
        -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/toolchain.cmake" \
        -DCMAKE_MAKE_PROGRAM="/usr/bin/make" \
        -DPython3_EXECUTABLE="${PYTHON_HOST_BIN}"; then
    echo -e "\033[1;31mError: Failed to cross-compile ROS 2 packages.\033[0m"
    exit 1
fi

# Helper function to normalize paths inside install/ for target deployment
normalize_target_paths() {
    local target_ws="$1"
    echo -e "\033[1;32mNormalizing target setup scripts for deployment...\033[0m"
    find "${WORKSPACE_DIR}/install" -type f \( -name "setup.*" -o -name "local_setup.*" -o -name "_local_setup_util*.py" -o -name "*prefix_path*" \) \
        -exec sed -i "s|${SYSROOT}||g; s|::|:|g" {} +
    if [ -n "${target_ws}" ] && [ "${target_ws}" != "${WORKSPACE_DIR}" ]; then
        echo -e "\033[1;32mAdapting install prefix to target workspace: ${target_ws}/install ...\033[0m"
        find "${WORKSPACE_DIR}/install" -type f \( -name "setup.*" -o -name "local_setup.*" -o -name "package.*" -o -name "*prefix_path*" \) \
            -exec sed -i "s|${WORKSPACE_DIR}/install|${target_ws}/install|g; s|${WORKSPACE_DIR}|${target_ws}|g" {} +
    fi
}

# Helper function to restore paths inside install/ back to host workspace
restore_host_paths() {
    local target_ws="$1"
    if [ -n "${target_ws}" ] && [ "${target_ws}" != "${WORKSPACE_DIR}" ]; then
        echo -e "\033[1;32mRestoring local host paths in install/ ...\033[0m"
        find "${WORKSPACE_DIR}/install" -type f \( -name "setup.*" -o -name "local_setup.*" -o -name "package.*" -o -name "*prefix_path*" \) \
            -exec sed -i "s|${target_ws}/install|${WORKSPACE_DIR}/install|g; s|${target_ws}|${WORKSPACE_DIR}|g" {} +
    fi
}

normalize_target_paths ""

echo -e "\033[1;32mCross compilation successful! Output is available in: ${WORKSPACE_DIR}/install\033[0m"

ARCHIVE_NAME="ugv_ws.tar.gz"
mkdir -p "${WORKSPACE_DIR}/log"

# Handle offline packaging mode
if [ "$1" == "pack" ]; then
    TARGET_PATH="${2:-${WORKSPACE_DIR}}"
    
    echo -e "\033[1;32m=== Packaging Complete Workspace (install + build + log + src) ===\033[0m"
    normalize_target_paths "${TARGET_PATH}"

    echo -e "\033[1;32mCreating archive: ${WORKSPACE_DIR}/${ARCHIVE_NAME} ...\033[0m"
    tar -czf "${WORKSPACE_DIR}/${ARCHIVE_NAME}" -C "${WORKSPACE_DIR}" install build log src

    restore_host_paths "${TARGET_PATH}"

    ARCHIVE_SIZE=$(du -h "${WORKSPACE_DIR}/${ARCHIVE_NAME}" | cut -f1)
    echo -e "\033[1;32mSuccessfully packaged full workspace into: ${WORKSPACE_DIR}/${ARCHIVE_NAME} (${ARCHIVE_SIZE})\033[0m"
    echo -e "\033[1;33mTip: You can copy this archive to your target device and extract it:\033[0m"
    echo -e "  mkdir -p ${TARGET_PATH} && tar -xzf ${ARCHIVE_NAME} -C ${TARGET_PATH}"
    exit 0
fi

# Handle remote deployment mode
if [ "$1" == "dest" ]; then
    PORT="$2"
    TARGET_SPEC="$3"

    if [ -z "${PORT}" ] || [ -z "${TARGET_SPEC}" ]; then
        echo -e "\033[1;31mError: Missing deployment arguments.\033[0m"
        echo -e "\033[1;33mUsage: $0 dest <port> <user>@<ip>:<remote_path>\033[0m"
        exit 1
    fi

    REMOTE_USER=$(echo "${TARGET_SPEC}" | cut -d'@' -f1)
    HOST=$(echo "${TARGET_SPEC}" | cut -d'@' -f2 | cut -d':' -f1)
    REMOTE_PATH=$(echo "${TARGET_SPEC}" | cut -d':' -f2)

    if [ -z "${REMOTE_USER}" ] || [ -z "${HOST}" ] || [ -z "${REMOTE_PATH}" ]; then
        echo -e "\033[1;31mError: Invalid target device format: '${TARGET_SPEC}'\033[0m"
        echo -e "\033[1;33mExpected format: username@host:/path/to/target\033[0m"
        exit 1
    fi

    # Automatically resolve leading '~' to absolute path (/home/<user> or /root)
    # This prevents tilde quote issues in shell expansion and ensures proper install prefix paths
    if [[ "${REMOTE_PATH}" == "~"* ]]; then
        if [ "${REMOTE_USER}" == "root" ]; then
            REMOTE_HOME="/root"
        else
            REMOTE_HOME="/home/${REMOTE_USER}"
        fi
        REMOTE_PATH="${REMOTE_HOME}${REMOTE_PATH:1}"
    fi

    echo -e "\033[1;32m=== Deploying Complete Workspace to ${REMOTE_USER}@${HOST}:${REMOTE_PATH} ===\033[0m"
    normalize_target_paths "${REMOTE_PATH}"

    echo -e "\033[1;32mCompressing full workspace (install + build + log + src) for deployment...\033[0m"
    tar -czf "${WORKSPACE_DIR}/${ARCHIVE_NAME}" -C "${WORKSPACE_DIR}" install build log src

    restore_host_paths "${REMOTE_PATH}"

    echo -e "\033[1;32mEnsuring remote directory exists at ${REMOTE_PATH}...\033[0m"
    if ! ssh -p "${PORT}" "${REMOTE_USER}@${HOST}" "mkdir -p '${REMOTE_PATH}'"; then
        echo -e "\033[1;31mError: Failed to create remote directory '${REMOTE_PATH}'. Check permissions and path.\033[0m"
        rm -f "${WORKSPACE_DIR}/${ARCHIVE_NAME}"
        exit 1
    fi

    echo -e "\033[1;32mTransferring archive to ${REMOTE_USER}@${HOST}:${REMOTE_PATH} (port ${PORT})...\033[0m"
    # Use 'scp -O' (legacy SCP protocol) to avoid SFTP subsystem failure ('subsystem request failed on channel 0')
    # Fallback to SSH stream if remote system lacks scp or SCP transfer fails
    if ! scp -O -P "${PORT}" "${WORKSPACE_DIR}/${ARCHIVE_NAME}" "${REMOTE_USER}@${HOST}:${REMOTE_PATH}/${ARCHIVE_NAME}"; then
        echo -e "\033[1;33mWarning: scp -O failed, falling back to SSH streaming transfer...\033[0m"
        if ! ssh -p "${PORT}" "${REMOTE_USER}@${HOST}" "cat > '${REMOTE_PATH}/${ARCHIVE_NAME}'" < "${WORKSPACE_DIR}/${ARCHIVE_NAME}"; then
            echo -e "\033[1;31mError: Transfer failed via both SCP and SSH stream.\033[0m"
            rm -f "${WORKSPACE_DIR}/${ARCHIVE_NAME}"
            exit 1
        fi
    fi

    echo -e "\033[1;32mExtracting archive on remote target at ${REMOTE_PATH}...\033[0m"
    if ! ssh -p "${PORT}" "${REMOTE_USER}@${HOST}" "cd '${REMOTE_PATH}' && tar -xzf '${ARCHIVE_NAME}' && rm -f '${ARCHIVE_NAME}'"; then
        echo -e "\033[1;31mError: Remote extraction failed.\033[0m"
        rm -f "${WORKSPACE_DIR}/${ARCHIVE_NAME}"
        exit 1
    fi

    rm -f "${WORKSPACE_DIR}/${ARCHIVE_NAME}"
    echo -e "\033[1;32mComplete workspace (install + build + log + src) successfully deployed to ${REMOTE_PATH}!\033[0m"
else
    echo -e "\033[1;33mNext Steps & Tips:\033[0m"
    echo -e "  1. Package complete workspace for offline deployment:"
    echo -e "     $0 pack [target_path]"
    echo -e "  2. Deploy complete workspace directly to target device:"
    echo -e "     $0 dest <port> <user>@<ip>:<remote_path>"
fi
