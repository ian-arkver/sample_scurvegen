/*
 * Helper class to read and write FPGA registers using SPI via /dev/spidev
 * 
 * Connections:
 * SPI1 to FPGA (/dev/spidev0.0) - to configure use:
 *      sudo /opt/nvidia/jetson-io/jetson-io.py
 * 
 * Breakout board
 * 
 * 1 blk    Gnd
 * 2 brn    mosi
 * 3 red    miso
 * 4 org    sck
 * 5 yel    csn
 * 6 grn    irq
 * 7 blu    spare
 * 8 vio    Gnd
 * 
 * Nano connector:
 * 
 * 19 brn   spi1 - mosi
 * 20 blk   Gnd
 * 21 red   spi1 - miso
 * 23 org   spi1 - sck
 * 24 yel   spi1 - cs0
 * 30 vio   Gnd
 * 31 grn   gpio_pz0 (gpio200) label: GPIO11
 * 33 blu   gpio_pe6 (gpio38)
 */

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "spifpga.h"

SpiFpga::SpiFpga(const char *dev)
{
    uint32_t mode = SPI_MODE_0;
    uint8_t bits = 16;
    uint32_t speed = 1000000;

    fd = open(dev, O_RDWR);
    if (fd < 0)
    {
        std::cerr << "Unable to open SPI device: " << dev << "\n";
        return;
    }

    if (ioctl(fd, SPI_IOC_WR_MODE32, &mode) == -1)
        throw std::runtime_error("Can't set spi mode");
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
        throw std::runtime_error("Can't set spi bits");
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
        throw std::runtime_error("Can't set spi speed");
}

SpiFpga::~SpiFpga()
{
    if (fd >= 0)
        close(fd);
}

uint16_t SpiFpga::SpiXfer(const uint16_t addrwrt, const uint16_t wval)
{
    uint16_t tx[2];
    uint16_t rx[2] = {0, 0};

    if (fd < 0)
        return 0;

    tx[0] = __builtin_bswap16(addrwrt);
    tx[1] = __builtin_bswap16(wval);

    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 4;
    tr.speed_hz = 500000;
    tr.bits_per_word = 16;
    tr.cs_change = 1;
    {
        // Serialise these in case of eg. irq handler reads
        std::lock_guard<std::mutex> lk(xfer_mtx);
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) == -1)
            throw std::runtime_error("SPI transfer failed");
    }

    uint16_t rv = __builtin_bswap16(rx[1]);

    auto st = saveLogState();
    LOG() << "SPI " << ((addrwrt & WRITE_CMD_BIT) ? "write" : "read")
          << " addr=" << std::hex << std::setfill('0') << std::setw(4) << (addrwrt >> ADDR_SHIFT)
          << " wr=" << wval << " rd=" << rv << std::endl;
    restoreLogState(st);
    return rv;
}

void SpiFpga::SpiWriteBulk(const uint16_t addr, const std::vector<uint16_t> wvals)
{
    int num_regs = wvals.size();
    uint16_t addrwrt = (addr << ADDR_SHIFT) | WRITE_CMD_BIT;
    uint16_t tx[1 + num_regs];

    if (fd < 0 || num_regs < 1 || num_regs > 32)
        return;

    tx[0] = __builtin_bswap16(addrwrt);
    for (int r = 0; r < num_regs; r++)
        tx[r + 1] = __builtin_bswap16(wvals[r]);

    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)tx;
    tr.len = 2 + num_regs * 2;
    tr.speed_hz = 500000;
    tr.bits_per_word = 16;
    tr.cs_change = 1;
    {
        // Serialise these in case of eg. irq handler reads
        std::lock_guard<std::mutex> lk(xfer_mtx);
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) == -1)
            throw std::runtime_error("SPI transfer failed");
    }

    // uint16_t rb = SpiReadReg(addr);

    auto st = saveLogState();
    LOG() << "SPI bulk write addr="
          << std::hex << std::setfill('0') << std::setw(4)
          << (addrwrt >> ADDR_SHIFT) << ": wr= ";
    for (const auto &wv : wvals)
        LOG() << wv << " ";
    // LOG() << ": rb= " << rb << std::endl;
    restoreLogState(st);
}

std::vector<uint16_t> SpiFpga::SpiReadBulk(const uint16_t addr, const std::size_t num)
{
    std::vector<uint16_t> rvals;
    uint16_t tx[1 + num];
    uint16_t rx[1 + num];

    if (fd < 0 || num < 1 || num > 32)
        return rvals;

    tx[0] = __builtin_bswap16(addr << ADDR_SHIFT);
    for (std::size_t r = 0; r < num; r++)
        tx[r + 1] = 0;

    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 2 + num * 2;
    tr.speed_hz = 500000;
    tr.bits_per_word = 16;
    tr.cs_change = 1;
    {
        // Serialise these in case of eg. irq handler reads
        std::lock_guard<std::mutex> lk(xfer_mtx);
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) == -1)
            throw std::runtime_error("SPI transfer failed");
    }

    for (std::size_t r = 0; r < num; r++)
        rvals.push_back(__builtin_bswap16(rx[r + 1]));

    auto st = saveLogState();
    LOG() << "SPI bulk read addr="
          << std::hex << std::setfill('0') << std::setw(4)
          << addr << ": wr= ";
    for (const auto &rv : rvals)
        LOG() << rv << " ";
    LOG() << "\n";
    restoreLogState(st);

    return rvals;
}

uint16_t SpiFpga::SpiRdWrtReg(const uint16_t addr, const uint16_t wval)
{
    return SpiXfer((addr << ADDR_SHIFT) | WRITE_CMD_BIT, wval);
}

void SpiFpga::SpiWriteReg(const uint16_t addr, const uint16_t val)
{
    (void)SpiXfer((addr << ADDR_SHIFT) | WRITE_CMD_BIT, val);
}

void SpiFpga::SpiMaskReg(const uint16_t addr, const uint16_t and_bits, const uint16_t or_bits)
{
    uint16_t tmp = SpiReadReg(addr);
    tmp = (tmp & and_bits) | or_bits;
    SpiWriteReg(addr, tmp);
}

uint16_t SpiFpga::SpiReadReg(const uint16_t addr)
{
    return SpiXfer(addr << ADDR_SHIFT, 0);
}

std::vector<uint16_t> SpiFpga::ConvertMoveToBulk(
    const axis::stepperMove move,
    uint16_t flags,
    bool start)
{
    std::vector<uint16_t> rv;

    if (move.total_steps >= 0)
        flags |= FR_STEPC_DIRJERK_DIR;

    rv.push_back((move.jerk & FR_STEPC_DIRJERK_JERK_MASK) | flags);
    rv.push_back(abs(move.total_steps) & 0xffff);
    rv.push_back(move.s_duration_us & 0xffff);
    rv.push_back(move.ka_duration_us & 0xffff);
    rv.push_back(
        ((move.ka_duration_us >> 16) & 0xf) << 8 |
        ((move.s_duration_us >> 16) & 0xf) << 4 |
        ((abs(move.total_steps) >> 16) & 0xf));
    if (start)
        rv.push_back(FR_STEPC_CMD_GO);
    return rv;
}
