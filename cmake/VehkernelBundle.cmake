# SPDX-License-Identifier: GPL-3.0-or-later
# Host tools bundle target (ADR-0026, spec 0014).
#
# Packages the host tools, generated metadata, and board firmware images into
# build/<preset>/bundle/ and build/<preset>/vehkernel-<preset>-<commit>.tar.gz.

if(NOT VK_PRESET)
  get_filename_component(VK_PRESET ${CMAKE_BINARY_DIR} NAME)
endif()

# The packer runs on every build: it is quick, and it reads the commit at build time, so
# the archive's name and manifest follow new commits without a new configure.
set(VK_BUNDLE_DIR ${CMAKE_BINARY_DIR}/bundle)

set(VK_BUNDLE_BOARD "")
if(VK_PLATFORM STREQUAL "stm32f4")
  set(VK_BUNDLE_BOARD ${VK_BOARD})
endif()

add_custom_target(bundle ALL
  COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_SOURCE_DIR}/tools/bundle/make_bundle.py
    --source-dir ${CMAKE_SOURCE_DIR}
    --build-dir ${CMAKE_BINARY_DIR}
    --preset ${VK_PRESET}
    --board "${VK_BUNDLE_BOARD}"
    --output-dir ${VK_BUNDLE_DIR}
  COMMENT "Packaging the host tools bundle"
  VERBATIM
)
if(VK_PLATFORM STREQUAL "stm32f4")
  add_dependencies(bundle vehkernel)
else()
  add_dependencies(bundle vk_msg_generate vk_params_generate)
endif()
