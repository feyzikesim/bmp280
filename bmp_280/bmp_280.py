from smbus2 import SMBus, i2c_msg
from time import sleep

def twos_complement(input):
    if input > 32768:
        return input - pow(2, 16)
    else:
        return input

class BMP280():
    BMP280_ADDR = 0x76

    t_fine = 0

    DIG_T1 = 0x88
    DIG_T2 = 0x8A
    DIG_T3 = 0x8C

    DIG_P1 = 0x8E
    DIG_P2 = 0x90
    DIG_P3 = 0x92
    DIG_P4 = 0x94
    DIG_P5 = 0x96
    DIG_P6 = 0x98
    DIG_P7 = 0x9A
    DIG_P8 = 0x9C
    DIG_P9 = 0x9E

    DEVICE_ID = 0xD0
    RESET = 0xE0
    STATUS = 0xF3
    CTRL_MEAS = 0xF4
    CONFIG = 0xF5
    PRESSURE = 0xF7
    TEMPERATURE = 0xFA

    RESET_CMD = 0xB6

    OVERSAMPLING_P_NONE = 0b000
    OVERSAMPLING_P_x1   = 0b001
    OVERSAMPLING_P_x2   = 0b010
    OVERSAMPLING_P_x4   = 0b011
    OVERSAMPLING_P_x8   = 0b100
    OVERSAMPLING_P_x16  = 0b101

    OVERSAMPLING_T_NONE = 0b000
    OVERSAMPLING_T_x1   = 0b001
    OVERSAMPLING_T_x2   = 0b010
    OVERSAMPLING_T_x4   = 0b011
    OVERSAMPLING_T_x8   = 0b100
    OVERSAMPLING_T_x16  = 0b101

    T_STANDBY_0p5 = 0b000
    T_STANDBY_62p5 = 0b001
    T_STANDBY_125 = 0b010
    T_STANDBY_250 = 0b011
    T_STANDBY_500 = 0b100
    T_STANDBY_1000 = 0b101
    T_STANDBY_2000 = 0b110
    T_STANDBY_4000 = 0b111

    IIR_FILTER_OFF = 0b000
    IIR_FILTER_x2  = 0b001
    IIR_FILTER_x4  = 0b010
    IIR_FILTER_x8  = 0b011
    IIR_FILTER_x16 = 0b100

    SLEEP_MODE = 0b00
    FORCED_MODE = 0b01
    NORMAL_MODE = 0b11

    def __init__(self, port, mode, oversampling_p, oversampling_t, filter, standby):
        self.bus = SMBus(port)
        self.bmp280_init(mode, oversampling_p, oversampling_t, filter, standby)

    def read_device_id(self):
        return self.bus.read_byte_data(self.BMP280_ADDR, self.DEVICE_ID)

    def device_reset(self):
        self.bus.write_byte_data(self.BMP280_ADDR, self.RESET, self.RESET_CMD)
        sleep(1)

    def bmp280_init(self, mode, oversampling_p, oversampling_t, filter, standby):
        ctrl_meas_reg = mode + (oversampling_p << 2) + (oversampling_t << 5)
        self.bus.write_byte_data(self.BMP280_ADDR, self.CTRL_MEAS, ctrl_meas_reg)

        config_reg = 0b000 + (filter << 2) + (standby << 5)
        self.bus.write_byte_data(self.BMP280_ADDR, self.CONFIG, config_reg)

    def read_temperature(self):
        t1 = self.bus.read_word_data(self.BMP280_ADDR, self.DIG_T1)
        t2 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_T2))
        t3 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_T3))

        raw_data = self.bus.read_i2c_block_data(self.BMP280_ADDR, self.TEMPERATURE, 3)
        adc_t = (raw_data[0] * pow(2, 16) + raw_data[1] * pow(2, 8) + raw_data[2]) >> 4

        var1 = ((adc_t / 16384.0) - (t1 / 1024.0)) * t2
        var2 = ((adc_t / 131072.0) - (t1 / 8192.0)) * (((adc_t / 131072.0) - (t1 / 8192.0)) * t3)
        self.t_fine = var1 + var2
        return (var1+var2) / 5120.0

    def read_pressure(self):
        p1 = self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P1)
        p2 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P2))
        p3 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P3))
        p4 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P4))
        p5 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P5))
        p6 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P6))
        p7 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P7))
        p8 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P8))
        p9 = twos_complement(self.bus.read_word_data(self.BMP280_ADDR, self.DIG_P9))

        raw_data = self.bus.read_i2c_block_data(self.BMP280_ADDR, self.PRESSURE, 3)
        adc_p = (raw_data[0] * pow(2, 16) + raw_data[1] * pow(2, 8) + raw_data[2]) >> 4

        self.read_temperature()

        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * p6 / 32768.0
        var2 += var1 * p5 * 2.0
        var2 = (var2 / 4.0) + (p4 * 65536.0)
        var1 = (p3 * var1 * var1 / 524288.0 + (p2 * var1)) / 524288.0
        var1 = (1.0 + (var1 / 32768.0)) * p1
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = p9 * p * p / 2147483648.0
        var2 = p * p8 / 32768.0
        p += (var1 + var2 + p7) / 16.0
        return p / 100
