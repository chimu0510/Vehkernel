#!/usr/bin/env bash
# 新環境的開發環境檢查與安裝（Ubuntu 24.04）。
#
#   tools/setup.sh                 檢查全部，列出要做的事，確認後安裝缺少的項目
#   tools/setup.sh --check         只檢查，不安裝；有缺少時以非 0 結束
#   tools/setup.sh --yes           不詢問，直接安裝
#   tools/setup.sh --only firmware 只處理 C50C 韌體開發（examples、燒錄）
#   tools/setup.sh --only ros      只處理 ROS2（主機工具、arm64 交叉編譯）
#
# 無法自動安裝的項目（需要登入 ST 網站或手動下載）只會檢查並提示。
# 可以重複執行；已經就緒的項目會略過。

set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CHECK_ONLY=0
ASSUME_YES=0
ONLY=""

usage() { sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check) CHECK_ONLY=1 ;;
        --yes|-y) ASSUME_YES=1 ;;
        --only) ONLY="${2:-}"; shift ;;
        -h|--help) usage 0 ;;
        *) echo "unknown option: $1" >&2; usage 1 ;;
    esac
    shift
done
case "$ONLY" in ""|firmware|ros) ;; *) echo "--only must be firmware or ros" >&2; exit 1 ;; esac

if [[ -t 1 ]]; then
    GREEN=$'\e[32m'; YELLOW=$'\e[33m'; RED=$'\e[31m'; BOLD=$'\e[1m'; RESET=$'\e[0m'
else
    GREEN=""; YELLOW=""; RED=""; BOLD=""; RESET=""
fi

APT_PACKAGES=()     # 要以 apt 安裝的套件
ACTIONS=()          # 其他要執行的動作："說明|指令"
MANUAL=()           # 需要使用者手動處理的項目
RELOGIN=0           # 加入群組後需要重新登入

ok()     { printf '  %s✔%s %s\n' "$GREEN" "$RESET" "$*"; }
todo()   { printf '  %s✘%s %s\n' "$RED" "$RESET" "$*"; }
note()   { printf '  %s!%s %s\n' "$YELLOW" "$RESET" "$*"; }
manual() { note "$*"; MANUAL+=("$*"); }
section(){ printf '\n%s== %s ==%s\n' "$BOLD" "$1" "$RESET"; }
have()   { command -v "$1" >/dev/null 2>&1; }
need_apt() { APT_PACKAGES+=("$@"); }
need_action() { ACTIONS+=("$1|$2"); }
in_group() { id -nG "$USER" | tr ' ' '\n' | grep -qx "$1"; }

version_ge() { [[ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -1)" == "$2" ]]; }

# --------------------------------------------------------------------------- 共用

check_os() {
    section "system"
    # shellcheck disable=SC1091
    . /etc/os-release
    if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]]; then
        ok "Ubuntu 24.04 (${PRETTY_NAME})"
    else
        manual "this script targets Ubuntu 24.04, found ${PRETTY_NAME:-unknown}; package names may differ"
    fi
    if have git; then ok "git $(git --version | awk '{print $3}')"; else todo "git"; need_apt git; fi
    if have curl; then ok "curl"; else todo "curl"; need_apt curl; fi
}

# --------------------------------------------------------------------------- 韌體

find_programmer() {
    if [[ -n "${STM32_PROGRAMMER_CLI:-}" && -x "${STM32_PROGRAMMER_CLI}" ]]; then
        echo "$STM32_PROGRAMMER_CLI"; return 0
    fi
    if have STM32_Programmer_CLI; then command -v STM32_Programmer_CLI; return 0; fi
    local p
    for p in "$HOME"/*/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
             /opt/st/*/STM32CubeProgrammer/bin/STM32_Programmer_CLI; do
        [[ -x "$p" ]] && { echo "$p"; return 0; }
    done
    return 1
}

check_firmware() {
    section "firmware (examples, flashing)"

    # 第三方原始碼：HAL、CMSIS
    local missing
    missing="$(git -C "$REPO" submodule status 2>/dev/null | grep -c '^-' || true)"
    if [[ "$missing" == "0" ]]; then
        ok "git submodules (third_party/ HAL, CMSIS)"
    else
        todo "git submodules: $missing not initialised"
        need_action "initialise git submodules" "git -C '$REPO' submodule update --init --depth 1"
    fi

    if have arm-none-eabi-gcc; then
        ok "arm-none-eabi-gcc $(arm-none-eabi-gcc -dumpversion) ($(command -v arm-none-eabi-gcc))"
    else
        todo "arm-none-eabi-gcc"
        need_apt gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
    fi

    if have cmake && version_ge "$(cmake --version | awk 'NR==1{print $3}')" 3.20; then
        ok "cmake $(cmake --version | awk 'NR==1{print $3}')"
    else
        todo "cmake >= 3.20"; need_apt cmake
    fi
    if have ninja; then ok "ninja $(ninja --version)"; else todo "ninja"; need_apt ninja-build; fi
    if have python3; then ok "python3 $(python3 -c 'import platform; print(platform.python_version())')"
    else todo "python3"; need_apt python3; fi
    if python3 -c 'import serial' 2>/dev/null; then
        ok "pyserial $(python3 -c 'import serial; print(serial.__version__)')"
    else
        todo "pyserial (tools/c50c.py)"; need_apt python3-serial
    fi
    if python3 -c 'import numpy, scipy' 2>/dev/null; then
        ok "numpy $(python3 -c 'import numpy; print(numpy.__version__)'), scipy $(python3 -c 'import scipy; print(scipy.__version__)')"
    else
        todo "numpy, scipy (tools/ahrs_bench)"; need_apt python3-numpy python3-scipy
    fi
    if have gcc; then ok "gcc $(gcc -dumpversion) (tools/ahrs_bench compiles the filters for the host)"
    else todo "gcc (tools/ahrs_bench)"; need_apt gcc; fi

    local cli
    if cli="$(find_programmer)"; then
        ok "STM32CubeProgrammer ($cli)"
    else
        manual "STM32CubeProgrammer: download from https://www.st.com/en/development-tools/stm32cubeprog.html" \
               "(needs an ST account), install it, then add its bin/ to PATH or set STM32_PROGRAMMER_CLI"
    fi

    # 串口權限與 ModemManager
    if in_group dialout; then
        ok "user '$USER' in group dialout (serial port access)"
    else
        todo "user '$USER' not in group dialout"
        need_action "add $USER to group dialout" "sudo usermod -aG dialout '$USER'"
        RELOGIN=1
    fi
    local rule=/etc/udev/rules.d/99-c50c-ch9102.rules
    if [[ -f "$rule" ]]; then
        ok "udev rule for CH9102F ($rule)"
    else
        todo "udev rule for CH9102F: keep ModemManager off /dev/ttyACM* while flashing"
        need_action "install $rule" \
            "printf '%s\n' '# C50C CH9102F USB serial: keep ModemManager away so it does not disturb the bootloader' 'SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ENV{ID_MM_DEVICE_IGNORE}=\"1\", GROUP=\"dialout\", MODE=\"0660\"' | sudo tee '$rule' >/dev/null && sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty"
    fi
}

build_examples() {
    section "build examples"
    if ! have arm-none-eabi-gcc || ! have cmake || ! have ninja ||
       [[ ! -f "$REPO/third_party/stm32f4xx_hal_driver/Inc/stm32f4xx_hal.h" ]]; then
        manual "examples not built: toolchain or submodules still missing"
        return
    fi
    if cmake -S "$REPO/examples" -B "$REPO/examples/build" -G Ninja >/dev/null &&
       cmake --build "$REPO/examples/build" >/dev/null; then
        ok "examples built: $(ls "$REPO"/examples/build/*.bin 2>/dev/null | wc -l) firmware images in examples/build/"
    else
        todo "examples failed to build; run: cmake --build examples/build"
    fi
}

# --------------------------------------------------------------------------- ROS2

check_ros() {
    section "ROS 2 (host tools, arm64 cross-compile)"

    if [[ -f /opt/ros/jazzy/setup.bash ]]; then
        ok "ROS 2 Jazzy (/opt/ros/jazzy)"
    else
        todo "ROS 2 Jazzy"
        need_action "add the ROS 2 apt source" \
            "v=\$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'\"' '{print \$4}') && curl -fsSL -o /tmp/ros2-apt-source.deb \"https://github.com/ros-infrastructure/ros-apt-source/releases/download/\$v/ros2-apt-source_\$v.\$(. /etc/os-release && echo \$VERSION_CODENAME)_all.deb\" && sudo dpkg -i /tmp/ros2-apt-source.deb && sudo apt-get update"
        need_action "install ROS 2 Jazzy desktop and dev tools" \
            "sudo apt-get install -y ros-jazzy-desktop ros-dev-tools"
    fi
    if have colcon; then ok "colcon"; elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
        todo "colcon"; need_apt ros-dev-tools
    fi

    # ugv_model_viewer 的建置依賴（ROS 套件以外的部分）
    if dpkg -s qtbase5-dev >/dev/null 2>&1; then ok "qtbase5-dev (ugv_model_viewer)"
    else todo "qtbase5-dev (ugv_model_viewer)"; need_apt qtbase5-dev; fi
    if python3 -c 'import numpy' 2>/dev/null; then ok "python3-numpy"
    else todo "python3-numpy"; need_apt python3-numpy; fi

    # arm64 交叉編譯（ros2_ws/scripts）
    if have aarch64-linux-gnu-gcc && have aarch64-linux-gnu-g++; then
        ok "aarch64-linux-gnu-gcc/g++ $(aarch64-linux-gnu-gcc -dumpversion)"
    else
        todo "aarch64 cross compiler"; need_apt gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
    fi
    if have pkg-config; then ok "pkg-config"; else todo "pkg-config"; need_apt pkg-config; fi
    if have ssh && have scp; then ok "ssh/scp (deploy to the Pi)"; else todo "ssh/scp"; need_apt openssh-client; fi
    if have docker; then
        ok "docker $(docker --version | awk '{print $3}' | tr -d ,)"
    else
        todo "docker (builds the arm64 sysroot)"; need_apt docker.io docker-buildx
    fi
    if docker buildx version >/dev/null 2>&1; then
        ok "docker buildx"
    else
        todo "docker buildx (BuildKit builder for docker)"; need_apt docker-buildx
    fi
    if in_group docker; then
        ok "user '$USER' in group docker"
    else
        todo "user '$USER' not in group docker"
        need_action "add $USER to group docker" "sudo groupadd -f docker && sudo usermod -aG docker '$USER'"
        RELOGIN=1
    fi

    if [[ -e "$REPO/ros2_ws/ubuntu_arm64" ]]; then
        ok "arm64 sysroot (ros2_ws/ubuntu_arm64)"
    elif [[ $RELOGIN -eq 1 ]] || ! docker info >/dev/null 2>&1; then
        manual "arm64 sysroot missing: after docker works without sudo (re-login), run" \
               "ros2_ws/scripts/setup_qemu_environment.sh (takes a while)"
    else
        todo "arm64 sysroot (ros2_ws/ubuntu_arm64)"
        need_action "build the arm64 sysroot with docker + QEMU (slow)" \
            "cd '$REPO/ros2_ws/scripts' && ./setup_qemu_environment.sh"
    fi

    if [[ -d "$REPO/extensions/wheeltec_robot_urdf/urdf" ]]; then
        ok "extension wheeltec_robot_urdf (chassis models)"
    else
        manual "extension wheeltec_robot_urdf missing: download it as described in extensions/README.md" \
               "(needed by ugv_model_viewer)"
    fi
}

# --------------------------------------------------------------------------- 主流程

run_plan() {
    local -a pkgs=()
    local item
    # 去除重複
    for item in "${APT_PACKAGES[@]}"; do
        [[ " ${pkgs[*]} " == *" $item "* ]] || pkgs+=("$item")
    done

    if [[ ${#pkgs[@]} -eq 0 && ${#ACTIONS[@]} -eq 0 ]]; then
        return 0
    fi
    section "plan"
    [[ ${#pkgs[@]} -gt 0 ]] && echo "  apt install: ${pkgs[*]}"
    for item in "${ACTIONS[@]}"; do echo "  ${item%%|*}"; done

    if [[ $CHECK_ONLY -eq 1 ]]; then
        return 1
    fi
    if [[ $ASSUME_YES -eq 0 ]]; then
        read -r -p "  proceed? (sudo will be used) [y/N] " reply
        [[ "$reply" =~ ^[Yy]$ ]] || { echo "  aborted"; return 1; }
    fi

    local failed=0
    if [[ ${#pkgs[@]} -gt 0 ]]; then
        sudo apt-get update && sudo apt-get install -y "${pkgs[@]}" || failed=1
    fi
    for item in "${ACTIONS[@]}"; do
        printf '\n%s> %s%s\n' "$BOLD" "${item%%|*}" "$RESET"
        bash -c "${item#*|}" || { todo "failed: ${item%%|*}"; failed=1; }
    done
    return $failed
}

main() {
    echo "${BOLD}ROS2-UGV environment setup${RESET} ($REPO)"
    check_os
    [[ -z "$ONLY" || "$ONLY" == firmware ]] && check_firmware
    [[ -z "$ONLY" || "$ONLY" == ros ]] && check_ros

    run_plan
    local status=$?

    if [[ $CHECK_ONLY -eq 0 && $status -eq 0 && ( -z "$ONLY" || "$ONLY" == firmware ) ]]; then
        build_examples
    fi

    section "summary"
    if [[ $status -ne 0 ]]; then
        todo "some items are still missing (see above)"
    elif [[ ${#APT_PACKAGES[@]} -gt 0 || ${#ACTIONS[@]} -gt 0 ]]; then
        ok "installed the missing items"
    else
        ok "everything that can be installed automatically is ready"
    fi
    for item in "${MANUAL[@]}"; do note "$item"; done
    if [[ $RELOGIN -eq 1 && $CHECK_ONLY -eq 0 ]]; then
        note "log out and back in so the new group memberships (dialout/docker) take effect, then re-run this script"
    fi
    [[ ${#MANUAL[@]} -eq 0 && $status -eq 0 ]]
}

main
