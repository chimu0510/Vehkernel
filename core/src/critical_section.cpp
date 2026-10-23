// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/core/critical_section.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <cstdlib>

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
    configASSERT(false);
    std::abort();  // configASSERT may be compiled out.
}

}  // namespace vk::core
