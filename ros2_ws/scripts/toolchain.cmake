# ROS 2 Cross-compilation Toolchain File for ARM64 (aarch64)
# Configured for portable builds without hardcoded absolute paths

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Strictly enforce using host tools for programs/compilers
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Target triple
set(TRIPLE ${CMAKE_SYSTEM_PROCESSOR}-linux-gnu)

# Host make / build tool (must run on host)
find_program(CMAKE_MAKE_PROGRAM NAMES make gmake PATHS /usr/bin /bin NO_DEFAULT_PATH REQUIRED)

# Cross-compilers (located via PATH or standard system locations)
find_program(CMAKE_C_COMPILER NAMES ${TRIPLE}-gcc PATHS /usr/bin /bin REQUIRED)
find_program(CMAKE_CXX_COMPILER NAMES ${TRIPLE}-g++ PATHS /usr/bin /bin REQUIRED)
find_program(CMAKE_LINKER NAMES ${TRIPLE}-ld PATHS /usr/bin /bin REQUIRED)

# Linker flags for ROS 2 shared libraries and executables
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

# Dynamically resolve paths relative to this toolchain file
get_filename_component(SCRIPTS_DIR "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
get_filename_component(WS_DIR "${SCRIPTS_DIR}/.." ABSOLUTE)

# Locate Sysroot (prefers ENV{SYSROOT}, otherwise auto-detects in workspace or scripts)
if(NOT DEFINED ENV{SYSROOT} OR "$ENV{SYSROOT}" STREQUAL "")
    if(EXISTS "${WS_DIR}/ubuntu_arm64")
        set(ENV{SYSROOT} "${WS_DIR}/ubuntu_arm64")
    elseif(EXISTS "${SCRIPTS_DIR}/ubuntu_arm64")
        set(ENV{SYSROOT} "${SCRIPTS_DIR}/ubuntu_arm64")
    else()
        message(FATAL_ERROR "SYSROOT environment variable is not defined and ubuntu_arm64 directory was not found in ${WS_DIR} or ${SCRIPTS_DIR}")
    endif()
endif()

set(CMAKE_SYSROOT $ENV{SYSROOT})

# Set ROS 2 Distribution
if(NOT DEFINED ENV{ROS_DISTRO} OR "$ENV{ROS_DISTRO}" STREQUAL "")
    set(ENV{ROS_DISTRO} "jazzy")
endif()

# Set ROS 2 Sysroot Install Path
if(NOT DEFINED ENV{ROS2_INSTALL_PATH} OR "$ENV{ROS2_INSTALL_PATH}" STREQUAL "")
    set(ENV{ROS2_INSTALL_PATH} "$ENV{SYSROOT}/opt/ros/$ENV{ROS_DISTRO}")
endif()

# Root search behavior for cross-compiling:
# Host tools (Python, generators) are executed on host; libraries/headers strictly from sysroot
set(CMAKE_FIND_ROOT_PATH $ENV{ROS2_INSTALL_PATH} $ENV{SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Configure pkg-config to search within the target sysroot
set(ENV{PKG_CONFIG_DIR} "")
set(ENV{PKG_CONFIG_LIBDIR} "$ENV{SYSROOT}/usr/lib/pkgconfig:$ENV{SYSROOT}/usr/lib/${TRIPLE}/pkgconfig:$ENV{SYSROOT}/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "$ENV{SYSROOT}")

# ROS 2 workspace prefix paths (for finding already built packages in workspace and sysroot)
if(DEFINED ENV{ROS_WS_INSTALL_PATH} AND NOT "$ENV{ROS_WS_INSTALL_PATH}" STREQUAL "")
    list(APPEND CMAKE_PREFIX_PATH $ENV{ROS_WS_INSTALL_PATH})
endif()
list(APPEND CMAKE_PREFIX_PATH $ENV{ROS2_INSTALL_PATH} $ENV{SYSROOT}/usr)

# Target Python ABI
if(DEFINED ENV{PYTHON_SOABI} AND NOT "$ENV{PYTHON_SOABI}" STREQUAL "")
    set(PYTHON_SOABI $ENV{PYTHON_SOABI})
else()
    set(PYTHON_SOABI "cpython-312-${TRIPLE}")
endif()

# Bypass CMake TRY_RUN checks during cross-compilation
set(THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE)
