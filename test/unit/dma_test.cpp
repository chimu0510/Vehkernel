// SPDX-License-Identifier: GPL-3.0-or-later
// DMA assignment types from the interface layer, as the board generator emits them.
#include <gtest/gtest.h>

#include <vk/hal/dma.hpp>

TEST(Dma, StreamCarriesControllerStreamChannelAndSharing)
{
    using UsartTx = vk::hal::DmaStream<vk::hal::DmaController::Dma2, 7, 4, false>;
    EXPECT_EQ(UsartTx::controller, vk::hal::DmaController::Dma2);
    EXPECT_EQ(UsartTx::stream, 7U);
    EXPECT_EQ(UsartTx::channel, 4U);
    EXPECT_FALSE(UsartTx::shared);
}
