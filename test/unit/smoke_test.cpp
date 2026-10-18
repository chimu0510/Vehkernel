// Placeholder until real units exist: proves the test preset builds and runs GoogleTest.
#include <gtest/gtest.h>

#include <array>
#include <span>

TEST(Smoke, Cxx20LibraryIsAvailable)
{
    constexpr std::array<int, 3> values{1, 2, 3};
    const std::span<const int> view{values};
    EXPECT_EQ(view.size(), 3U);
}
