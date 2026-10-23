# SPDX-License-Identifier: GPL-3.0-or-later
# Scratch project for the layer rule tests: links layer VK_FROM to VK_TO.
cmake_minimum_required(VERSION 3.25)
project(VehkernelLayerRules NONE)

include(${VK_SOURCE_DIR}/cmake/VehkernelLayers.cmake)

foreach(layer IN LISTS VK_LAYERS)
  vk_add_library(vk_${layer} LAYER ${layer} INTERFACE)
endforeach()

vk_target_link_libraries(vk_${VK_FROM} INTERFACE vk_${VK_TO})
