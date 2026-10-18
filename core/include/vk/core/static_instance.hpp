// Storage for an object created once at start-up and never destroyed.
//
// Firmware objects live for the whole run, and main()'s stack is reused once
// the scheduler starts, so long-lived objects go into static storage. A
// plain static object would register an exit-time destructor; this does not.
#pragma once

#include <array>
#include <cstddef>
#include <new>
#include <utility>

namespace vk::core {

template <class T>
class StaticInstance {
public:
    // Creates the object; call at most once.
    template <class... Args>
    T& emplace(Args&&... args)
    {
        object_ = new (storage_.data()) T(std::forward<Args>(args)...);
        return *object_;
    }

    [[nodiscard]] bool has_value() const { return object_ != nullptr; }
    T& operator*() { return *object_; }
    T* operator->() { return object_; }

private:
    alignas(T) std::array<std::byte, sizeof(T)> storage_{};
    T* object_ = nullptr;
};

}  // namespace vk::core
