// SPDX-License-Identifier: GPL-3.0-or-later
// Start-up steps shared between the STM32F4 platform sources.
#pragma once

namespace vk::platform::detail {

void configure_clock_tree();
void start_microsecond_clock();

}  // namespace vk::platform::detail
