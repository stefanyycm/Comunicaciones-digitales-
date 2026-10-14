# nrf24.py - Driver NRF24L01 para Raspberry Pi Pico

import time
from machine import Pin


# Comandos SPI
CMD_R_REG = 0x00
CMD_W_REG = 0x20
CMD_RX = 0x61
CMD_TX = 0xA0
CMD_FLUSH_TX = 0xE1
CMD_FLUSH_RX = 0xE2
CMD_NOP = 0xFF


# Registros
REG_CONFIG = 0x00
REG_EN_AA = 0x01
REG_RX_ADDR = 0x02
REG_ADDR_WIDTH = 0x03
REG_RETR = 0x04
REG_CHANNEL = 0x05
REG_RF_SETUP = 0x06
REG_STATUS = 0x07
REG_RX_ADDR_P0 = 0x0A
REG_TX_ADDR = 0x10
REG_RX_WIDTH = 0x11
REG_FIFO = 0x17


MAX_PAYLOAD = 32
PIPE_ADDRESS = b"UMNG1"

POWER_DBM = (-18, -12, -6, 0)

RATES = (
    "250 kbps",
    "1 Mbps",
    "2 Mbps"
)

RATE_BITS = (
    0x20,
    0x00,
    0x08
)

BASE_CONFIG = 0x0E


class NRF24:

    def __init__(
        self,
        spi,
        csn,
        ce,
        channel=40,
        power=3,
        rate=1,
        auto_ack=True
    ):

        self.spi = spi

        self.csn = Pin(
            csn,
            Pin.OUT,
            value=1
        )

        self.ce = Pin(
            ce,
            Pin.OUT,
            value=0
        )

        self.channel = channel
        self.power = power
        self.rate = rate

        time.sleep_ms(100)

        self.write_reg(
            REG_CONFIG,
            BASE_CONFIG
        )

        time.sleep_ms(5)

        self.write_reg(
            REG_ADDR_WIDTH,
            0x03
        )

        self.write_reg(
            REG_RETR,
            0x5F
        )

        self.write_reg(
            REG_EN_AA,
            0x01 if auto_ack else 0
        )

        self.write_reg(
            REG_RX_ADDR,
            0x01
        )

        self.write_reg(
            REG_RX_ADDR_P0,
            PIPE_ADDRESS
        )

        self.write_reg(
            REG_TX_ADDR,
            PIPE_ADDRESS
        )

        self.write_reg(
            REG_RX_WIDTH,
            MAX_PAYLOAD
        )

        self.write_reg(
            REG_CHANNEL,
            channel
        )

        self._apply_rf()

        self._command(CMD_FLUSH_TX)
        self._command(CMD_FLUSH_RX)

        self.write_reg(
            REG_STATUS,
            0x70
        )


    def _command(
        self,
        command,
        data=b"",
        read_count=0
    ):

        tx = bytearray((command,))
        tx.extend(data)
        tx.extend(bytearray(read_count))

        rx = bytearray(len(tx))

        self.csn.value(0)

        self.spi.write_readinto(
            tx,
            rx
        )

        self.csn.value(1)

        return rx[0], rx[1:]


    def read_reg(
        self,
        register,
        amount=1
    ):

        return self._command(
            CMD_R_REG | register,
            read_count=amount
        )[1]


    def write_reg(
        self,
        register,
        value
    ):

        if isinstance(value, int):
            value = bytes((value,))

        self._command(
            CMD_W_REG | register,
            value
        )


    def _apply_rf(self):

        setup = (
            RATE_BITS[self.rate]
            | (self.power << 1)
        )

        self.write_reg(
            REG_RF_SETUP,
            setup
        )


    def set_channel(self, value):

        self.channel = value

        self.write_reg(
            REG_CHANNEL,
            value
        )

        return self.read_reg(
            REG_CHANNEL
        )[0]


    def set_power(self, value):

        self.power = value

        self._apply_rf()

        return self.read_reg(
            REG_RF_SETUP
        )[0]


    @property
    def freq_mhz(self):
        return 2400 + self.channel


    def send(self, payload):

        if len(payload) > MAX_PAYLOAD:
            return False

        packet = (
            bytes(payload)
            + bytes(MAX_PAYLOAD - len(payload))
        )

        self._command(
            CMD_TX,
            packet
        )

        self.ce.value(1)

        time.sleep_us(20)

        self.ce.value(0)

        status = 0
        start = time.ticks_ms()

        while time.ticks_diff(
            time.ticks_ms(),
            start
        ) < 200:

            status = self._command(
                CMD_NOP
            )[0]

            if status & 0x30:
                break

        self.write_reg(
            REG_STATUS,
            0x70
        )

        if status & 0x10:

            self._command(
                CMD_FLUSH_TX
            )

            return False

        return bool(
            status & 0x20
        )


    def start_listening(self):

        self.write_reg(
            REG_CONFIG,
            BASE_CONFIG | 0x01
        )

        self.write_reg(
            REG_STATUS,
            0x70
        )

        self._command(
            CMD_FLUSH_RX
        )

        self.ce.value(1)

        time.sleep_us(150)


    def stop_listening(self):

        self.ce.value(0)

        self.write_reg(
            REG_CONFIG,
            BASE_CONFIG
        )


    def any(self):

        return not bool(
            self.read_reg(REG_FIFO)[0] & 0x01
        )


    def recv(self):

        payload = self._command(
            CMD_RX,
            read_count=MAX_PAYLOAD
        )[1]

        self.write_reg(
            REG_STATUS,
            0x40
        )

        return bytes(payload)
