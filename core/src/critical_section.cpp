// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/core/critical_section.hpp>

#include <vk/core/fault.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <cstdint>

namespace vk::core {

CriticalSection::CriticalSection()
{
    taskENTER_CRITICAL();
}

CriticalSection::~CriticalSection()
{
    taskEXIT_CRITICAL();
}

void fail(const char* /*what*/)
{
    // The caller's address, for the reset report; `what` is for a debugger.
    fault_stop(Fault::Fail, static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(__builtin_return_address(0))));
}

}  // namespace vk::core
