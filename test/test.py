# SPDX-FileCopyrightText: (c) 2026 Enzo Nappi
# SPDX-License-Identifier: Apache-2.0
#
# End-to-end cocotb test: bit-bangs a synthetic SENT frame into ui_in[0],
# waits for it to be decoded, then bit-bangs an I2C master transaction (as
# the SMBus host would) against the register map and checks the results.
# Mirrors test/tb_sent_i2c.v (the Icarus-based RTL regression used during
# development) but driven from Python/cocotb against the packaged top level.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

CLK_PERIOD_NS = 100  # 10 MHz, matches the design's clk_hz assumption
TICK_CYCLES = 30  # 3 us tick @ 10 MHz -> inside the default 1-6us sync window
TICKS_SYNC = 56
NIBBLE_MIN_TICKS = 12
I2C_ADDR = 0x50


def crc4(nibbles):
    crc = 0x5
    for n in nibbles:
        c = crc ^ n
        for _ in range(4):
            c = ((c << 1) ^ 0x3) & 0xF if (c & 0x8) else (c << 1) & 0xF
        crc = c
    return crc


def _safe_int(logic_array):
    """int(logic_array), but treat unresolved ('x'/'z', e.g. before reset)
    bits as 0 instead of raising."""
    try:
        return int(logic_array)
    except ValueError:
        return 0


class Bus:
    """Models the open-drain SENT + I2C wiring between the testbench (as
    SENT source + I2C master) and the DUT, the same way tb_sent_i2c.v does:
    each side can only pull its line low, never drive it high."""

    def __init__(self, dut):
        self.dut = dut
        self.sent_drive_low = False
        self.scl_drive_low = False
        self.sda_drive_low = False

    def _dut_pulls_sda_low(self):
        return _safe_int(self.dut.uio_oe.value) & 0b10 and not (
            _safe_int(self.dut.uio_out.value) & 0b10
        )

    def apply(self):
        # ui_in[0] = SENT line: testbench is the only driver in this test
        self.dut.ui_in.value = 0 if self.sent_drive_low else 1
        # uio_in[0] = SCL: design never drives it (input only)
        scl = 0 if self.scl_drive_low else 1
        # uio_in[1] = SDA: wired-AND between us and the DUT's sda_oe
        sda = 0 if (self.sda_drive_low or self._dut_pulls_sda_low()) else 1
        self.dut.uio_in.value = (sda << 1) | scl

    def read_sda(self):
        return 0 if (self.sda_drive_low or self._dut_pulls_sda_low()) else 1

    async def run(self):
        # The DUT's own sda_oe changes on its own clock edges, independently
        # of when the testbench happens to call apply() -- a real open-drain
        # bus resolves continuously, so re-apply frequently instead of only
        # at the specific moments the testbench changes its own drive.
        while True:
            self.apply()
            await Timer(1, unit="ns")


async def sent_pulse(dut, bus, total_cycles, low_cycles=5):
    bus.sent_drive_low = True
    bus.apply()
    await ClockCycles(dut.clk, low_cycles)
    bus.sent_drive_low = False
    bus.apply()
    await ClockCycles(dut.clk, total_cycles - low_cycles)


async def send_sent_frame(dut, bus, status, data_nibbles):
    crc = crc4([status] + list(data_nibbles))
    await sent_pulse(dut, bus, TICKS_SYNC * TICK_CYCLES)
    for nib in [status] + list(data_nibbles) + [crc]:
        await sent_pulse(dut, bus, (NIBBLE_MIN_TICKS + nib) * TICK_CYCLES)


async def i2c_delay(dut):
    await ClockCycles(dut.clk, 8)


async def i2c_start(dut, bus):
    bus.scl_drive_low = True
    bus.sda_drive_low = False
    bus.apply()
    await i2c_delay(dut)
    bus.scl_drive_low = False
    bus.apply()
    await i2c_delay(dut)
    bus.sda_drive_low = True  # SDA falls while SCL high -> START
    bus.apply()
    await i2c_delay(dut)
    bus.scl_drive_low = True
    bus.apply()
    await i2c_delay(dut)


async def i2c_stop(dut, bus):
    bus.sda_drive_low = True
    bus.scl_drive_low = True
    bus.apply()
    await i2c_delay(dut)
    bus.scl_drive_low = False
    bus.apply()
    await i2c_delay(dut)
    bus.sda_drive_low = False  # SDA rises while SCL high -> STOP
    bus.apply()
    await i2c_delay(dut)


async def i2c_write_byte(dut, bus, data):
    for i in range(7, -1, -1):
        bus.sda_drive_low = not ((data >> i) & 1)
        bus.apply()
        await i2c_delay(dut)
        bus.scl_drive_low = False
        bus.apply()
        await i2c_delay(dut)
        bus.scl_drive_low = True
        bus.apply()
    bus.sda_drive_low = False  # release SDA so the slave can drive ack/nack
    bus.apply()
    await i2c_delay(dut)
    bus.scl_drive_low = False
    bus.apply()
    await i2c_delay(dut)
    ack = bus.read_sda() == 0
    bus.scl_drive_low = True
    bus.apply()
    await i2c_delay(dut)
    return ack


async def i2c_read_byte(dut, bus, nack):
    data = 0
    for _ in range(8):
        bus.sda_drive_low = False
        bus.apply()
        await i2c_delay(dut)
        bus.scl_drive_low = False
        bus.apply()
        await i2c_delay(dut)
        data = (data << 1) | bus.read_sda()
        bus.scl_drive_low = True
        bus.apply()
    bus.sda_drive_low = not nack  # NACK=release(1), ACK=pull low(0)
    bus.apply()
    await i2c_delay(dut)
    bus.scl_drive_low = False
    bus.apply()
    await i2c_delay(dut)
    bus.scl_drive_low = True
    bus.apply()
    return data


async def i2c_write_reg(dut, bus, ptr, value=None):
    await i2c_start(dut, bus)
    await i2c_write_byte(dut, bus, (I2C_ADDR << 1) | 0)
    await i2c_write_byte(dut, bus, ptr)
    if value is not None:
        await i2c_write_byte(dut, bus, value)
    await i2c_stop(dut, bus)


async def i2c_read_block(dut, bus, ptr, n):
    await i2c_write_reg(dut, bus, ptr)
    await i2c_start(dut, bus)  # repeated start
    await i2c_write_byte(dut, bus, (I2C_ADDR << 1) | 1)
    out = []
    for i in range(n):
        out.append(await i2c_read_byte(dut, bus, nack=(i == n - 1)))
    await i2c_stop(dut, bus)
    return out


@cocotb.test()
async def test_sent_i2c_bridge(dut):
    dut._log.info("Start")

    clock = Clock(dut.clk, CLK_PERIOD_NS, unit="ns")
    cocotb.start_soon(clock.start())

    bus = Bus(dut)
    dut.ena.value = 1
    dut.ui_in.value = 1  # SENT idle-high
    dut.uio_in.value = 0b11  # SCL/SDA idle-high (pulled up)
    cocotb.start_soon(bus.run())
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    # --- decode a synthetic SENT frame ---
    status = 0x7
    data_nibbles = [0x1, 0x2, 0x3, 0x4, 0x5, 0x6]
    await send_sent_frame(dut, bus, status, data_nibbles)
    # the decoder times each nibble (including CRC) by the falling edge that
    # *follows* it, so it needs one more edge after the frame before it can
    # latch -- a real bus would already be starting its next sync pulse here.
    await sent_pulse(dut, bus, TICKS_SYNC * TICK_CYCLES)
    await ClockCycles(dut.clk, 20)

    assert int(dut.uo_out.value) & 0b0001, "new_data should be set after a decoded frame"

    # --- read the frame back over I2C ---
    regs = await i2c_read_block(dut, bus, 0x00, 7)
    dut._log.info(f"regs = {[hex(r) for r in regs]}")

    status_reg, status_nibble, data0, data1, data2, crc, frame_count = regs

    assert status_nibble == status, f"STATUS_NIBBLE mismatch: {status_nibble:#x} != {status:#x}"
    expected_data0 = data_nibbles[0] | (data_nibbles[1] << 4)
    expected_data1 = data_nibbles[2] | (data_nibbles[3] << 4)
    expected_data2 = data_nibbles[4] | (data_nibbles[5] << 4)
    assert data0 == expected_data0, f"DATA0 mismatch: {data0:#x} != {expected_data0:#x}"
    assert data1 == expected_data1, f"DATA1 mismatch: {data1:#x} != {expected_data1:#x}"
    assert data2 == expected_data2, f"DATA2 mismatch: {data2:#x} != {expected_data2:#x}"
    assert crc == crc4([status] + data_nibbles), "CRC mismatch"
    assert (status_reg & 0b010) == 0, "crc_error should not be set on a valid frame"
    assert frame_count == 1, f"FRAME_COUNT mismatch: {frame_count}"

    dut._log.info("SENT frame decoded and read back correctly over I2C")

    # --- write + read back the runtime-configurable SYNC_MIN/MAX registers ---
    sync_min, sync_max = 6720, 10080
    await i2c_write_reg(dut, bus, 0x10, sync_min & 0xFF)
    await i2c_write_reg(dut, bus, 0x11, (sync_min >> 8) & 0xFF)
    await i2c_write_reg(dut, bus, 0x12, sync_max & 0xFF)
    await i2c_write_reg(dut, bus, 0x13, (sync_max >> 8) & 0xFF)

    rb = await i2c_read_block(dut, bus, 0x10, 4)
    rb_min = rb[0] | (rb[1] << 8)
    rb_max = rb[2] | (rb[3] << 8)
    assert rb_min == sync_min, f"SYNC_MIN readback mismatch: {rb_min} != {sync_min}"
    assert rb_max == sync_max, f"SYNC_MAX readback mismatch: {rb_max} != {sync_max}"

    dut._log.info("SYNC_MIN/MAX registers written and read back correctly")
