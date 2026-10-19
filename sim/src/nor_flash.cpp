#include <vk/sim/nor_flash.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <utility>

namespace vk::sim {
namespace {

constexpr std::uint32_t erased = 0xFFFFFFFFU;

}  // namespace

NorFlash::NorFlash(std::size_t sector_count, std::size_t sector_size)
    : sector_count_(sector_count), sector_size_(sector_size)
{
    words_.fill(erased);
}

NorFlash::~NorFlash()
{
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool NorFlash::survive()
{
    if (!powered_) {
        return false;
    }
    if (power_loss_in_ != 0) {
        --power_loss_in_;
        if (power_loss_in_ == 0) {
            powered_ = false;
            return false;
        }
    }
    ++operations_;
    return true;
}

bool NorFlash::erase(std::size_t sector)
{
    if (sector >= sector_count_) {
        return false;
    }
    const std::size_t first = sector * sector_size_ / 4;
    const std::size_t words = sector_size_ / 4;
    const bool was_powered = powered_;
    if (!survive()) {
        if (!was_powered) {
            return false;
        }
        // Lost during the erase: half of the sector is erased.
        for (std::size_t i = 0; i < words / 2; ++i) {
            words_[first + i] = erased;
        }
        save(first, words / 2);
        return false;
    }
    for (std::size_t i = 0; i < words; ++i) {
        words_[first + i] = erased;
    }
    save(first, words);
    return true;
}

hal::FlashResult NorFlash::program(std::size_t sector, std::size_t offset, std::uint32_t word)
{
    if (sector >= sector_count_ || offset % 4 != 0 || offset >= sector_size_) {
        return hal::FlashResult::Error;
    }
    const std::size_t index = (sector * sector_size_ + offset) / 4;
    if (words_[index] != erased) {
        return hal::FlashResult::NotErased;
    }
    const bool was_powered = powered_;
    if (!survive()) {
        if (was_powered) {
            // Lost during the write: only the low half of the word is programmed.
            words_[index] &= word | 0xFFFF0000U;
            save(index, 1);
        }
        return hal::FlashResult::Error;
    }
    words_[index] &= word;
    save(index, 1);
    return hal::FlashResult::Ok;
}

std::uint32_t NorFlash::read(std::size_t sector, std::size_t offset) const
{
    if (sector >= sector_count_ || offset % 4 != 0 || offset >= sector_size_) {
        return erased;
    }
    return words_[(sector * sector_size_ + offset) / 4];
}

bool NorFlash::attach_file(const char* path)
{
    const int fd = open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        return false;
    }
    const std::size_t bytes = sector_count_ * sector_size_;
    struct stat info {};
    if (fstat(fd, &info) == 0 && std::cmp_equal(info.st_size, bytes) &&
        std::cmp_equal(pread(fd, words_.data(), bytes, 0), bytes)) {
        fd_ = fd;
        return true;
    }
    words_.fill(erased);
    if (ftruncate(fd, 0) != 0 || !std::cmp_equal(pwrite(fd, words_.data(), bytes, 0), bytes)) {
        close(fd);
        return false;
    }
    fd_ = fd;
    return true;
}

void NorFlash::save(std::size_t first_word, std::size_t words)
{
    if (fd_ < 0) {
        return;
    }
    const auto offset = static_cast<off_t>(first_word * 4);
    // A failed write leaves the file behind the model; the next run starts from what it holds.
    (void)pwrite(fd_, &words_[first_word], words * 4, offset);
}

}  // namespace vk::sim
