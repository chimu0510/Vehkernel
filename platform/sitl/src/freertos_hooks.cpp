// FreeRTOS application hooks for SITL (ADR-0004, ADR-0018).
#include <FreeRTOS.h>
#include <task.h>

#include <array>

namespace {

StaticTask_t idle_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> idle_stack;

}  // namespace

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t** tcb, StackType_t** stack,
                                              configSTACK_DEPTH_TYPE* stack_size)
{
    *tcb = &idle_tcb;
    *stack = idle_stack.data();
    *stack_size = idle_stack.size();
}
