/*
 * Helper class to read/write regs in the FPGA via SPI.
 * Also includes the FPGA register definitions.
 */
#pragma once

#define FR_MISC_CONTROL 0x000
#define F_MISCCTRL_MASK 0x1f
#define F_MISCCTRL_LED_UP (1U << 0)
#define F_MISCCTRL_LED_DOWN (1U << 1)
#define F_MISCCTRL_PUMP_VAC (1U << 2)
#define F_MISCCTRL_POWER34V (1U << 3)
#define F_MISCCTRL_PANELSTOP (1U << 8)
#define F_MISCCTRL_PANELGO (1U << 9)

#define FR_DEVICE_ID 0x001
#define FR_DEVICE_REV 0x002

#define FR_IRQ_MASK 0x004
#define FR_IRQ_STATUS 0x005
#define F_IRQBIT_STEPDONE_X (1U << 0)
#define F_IRQBIT_STEPDONE_Y (1U << 1)
#define F_IRQBIT_STEPDONE_P (1U << 2)
#define F_IRQBIT_STEPDONE_H1 (1U << 3)
#define F_IRQBIT_STEPDONE_H2 (1U << 4)
#define F_IRQBIT_STEPDONE_XY (F_IRQBIT_STEPDONE_X | F_IRQBIT_STEPDONE_Y)
#define F_IRQBIT_STEPDONE(axis) (1U << axis)
#define F_IRQBIT_STEPDONE_ALL (0x1f)
#define F_IRQBIT_TEST (1U << 15)

#define FR_IRQ_TEST_ESTOP 0x006
#define F_IRQESTOP_TESTBIT (1U << 0)
#define F_IRQESTOP_STOPVAL 0xE000
#define F_IRQESTOP_RESETVAL 0x9000

#define FR_VALVE_CONTROL 0x008
#define F_VALVECTL_MASK 0x0f
#define F_VALVECTL_H1VAC (1U << 0)
#define F_VALVECTL_H1AIR (1U << 1)
#define F_VALVECTL_H2VAC (1U << 2)
#define F_VALVECTL_H2AIR (1U << 3)

#define FR_STOP_ALARMS 0x010
#define F_AXIS_X_ZERO (1U << 0)
#define F_AXIS_Y_ZERO (1U << 1)
#define F_HROT_H1_ZERO (1U << 2)
#define F_HROT_H2_ZERO (1U << 3)
#define F_HPUSH_H1_ZERO (1U << 4)
#define F_HPUSH_H2_ZERO (1U << 5)
#define F_AXIS_X_ALARM (1U << 6)
#define F_AXIS_Y_ALARM (1U << 7)

#define FR_HPRSENS_I2C_CTRL 0x018
#define F_HPR_CTL_CHIPSEL (1U << 8)
#define F_HPR_CTL_RDBURST (1U << 9)
#define F_HPR_CTL_WRITE (1U << 10)
#define F_HPR_CTL_START (1U << 12)
#define F_HPR_CTL_BUSY (1U << 15)

#define FR_HPRSENS_I2C_DATA0 0x019
#define FR_HPRSENS_I2C_DATA1 0x01A
#define FR_HPRSENS_I2C_DATA2 0x01B

#define F_CLR_SHIFT 8

#define FR_STEPCTRL_BASE(ch) (0x040 + ((ch) << 4))
#define FR_STEPCTRL_DIR_JERK(ch) (FR_STEPCTRL_BASE(ch) + 0)
#define FR_STEPCTRL_TOTALSTEPS(ch) (FR_STEPCTRL_BASE(ch) + 1)
#define FR_STEPCTRL_STEPSLEFT(ch) (FR_STEPCTRL_BASE(ch) + 2)
#define FR_STEPCTRL_CJ_DUR_16US(ch) (FR_STEPCTRL_BASE(ch) + 2)
#define FR_STEPCTRL_CA_DUR_16US(ch) (FR_STEPCTRL_BASE(ch) + 3)
#define FR_STEPCTRL_STEPSHI(ch) (FR_STEPCTRL_BASE(ch) + 4)
#define FR_STEPCTRL_CMD(ch) (FR_STEPCTRL_BASE(ch) + 5)

#define F_STEPSLEFT(hival, loval) (((((int)hival >> 4) & 0xf) << 16) | (int)loval)

#define FR_STEPC_DIRJERK_JERK_MASK (0x1FF)
#define FR_STEPC_DIRJERK_DIR (1U << 9)
#define FR_STEPC_DIRJERK_ZEROCLEAR (1U << 10)
#define FR_STEPC_DIRJERK_ZEROENAB (1U << 11)
#define FR_STEPC_CMD_GO (1U << 0)
#define FR_STEPC_CMD_BUSY (1U << 15)

#include <mutex>
#include <vector>

#include "axis.h"
#include "loggable.h"

class SpiFpga : public LogStream
{
public:
    SpiFpga(const char *dev);
    ~SpiFpga();

    uint16_t SpiReadReg(const uint16_t addr);
    void SpiWriteReg(const uint16_t addr, const uint16_t val);
    void SpiMaskReg(const uint16_t addr, const uint16_t and_bits, const uint16_t or_bits);
    // Returns old value
    uint16_t SpiRdWrtReg(const uint16_t addr, const uint16_t val);
    void SpiWriteBulk(const uint16_t addr, const std::vector<uint16_t> wvals);
    std::vector<uint16_t> SpiReadBulk(const uint16_t addr, const std::size_t num);

    std::vector<uint16_t> ConvertMoveToBulk(
        const axis::stepperMove move,
        uint16_t flags = 0, bool start = false);

    bool is_open()
    {
        return fd >= 0;
    }

private:
    std::mutex xfer_mtx;
    uint16_t SpiXfer(const uint16_t addrwrt, const uint16_t wval);

    static const uint16_t WRITE_CMD_BIT = 0x0008;
    static const uint16_t ADDR_MASK = 0xfff;
    static const int ADDR_SHIFT = 4;
    int fd;
};
