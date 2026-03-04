#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smbus2
import time
import math

# ======================== НАСТРАИВАЕМЫЕ ПАРАМЕТРЫ ========================
GYRO_SIGN = 1          # 1 - нормально, -1 - инвертировать все оси
MAG_OPTION = 1         # 1: X=-Y, Y=+X, Z=Z (стандарт)
                       # 2: X=+X, Y=+Y, Z=Z (без перестановки)
                       # 3: X=+Y, Y=-X, Z=Z (альтернатива)
USE_INIT_QUAT = True   # инициализировать кватернион по первым измерениям
BETA = 1.0             # коэффициент фильтра (1.0 - быстро)
# ========================================================================

BMA_ADDR = 0x19
BMG_ADDR = 0x69
BMM_ADDR = 0x13

# ----- Акселерометр (BMA) -----
BMA_CHIP_ID     = 0x00
BMA_ACC_DATA    = 0x02
BMA_RESET       = 0x14
BMA_PMU_LPW     = 0x11
BMA_ACC_RANGE   = 0x0F
BMA_ACC_BW      = 0x10
BMA_D_HBW       = 0x13
BMA_RANGE_2G    = 0x03
BMA_BW_125HZ    = 0x0C
BMA_ID_EXPECTED = 0xFA

# ----- Гироскоп (BMG) -----
BMG_CHIP_ID      = 0x00
BMG_DATA         = 0x02
BMG_RESET        = 0x14
BMG_LPM1         = 0x11
BMG_RANGE        = 0x0F
BMG_BW           = 0x10
BMG_D_HBW        = 0x13
BMG_RANGE_125DPS = 0x04
BMG_BW_23HZ      = 0x04
BMG_ID_EXPECTED  = 0x0F
BMG_SCALE_125    = 124.87 / 32768.0

# ----- Магнитометр (BMM) -----
BMM_CHIP_ID     = 0x40
BMM_POWER       = 0x4B
BMM_DATA        = 0x42
BMM_CTRL1       = 0x4C
BMM_REP_XY      = 0x51
BMM_REP_Z       = 0x52
BMM_DIG_X1      = 0x5D
BMM_DIG_Y1      = 0x5E
BMM_DIG_Z4_LSB  = 0x62
BMM_DIG_Z4_MSB  = 0x63
BMM_DIG_X2      = 0x64
BMM_DIG_Y2      = 0x65
BMM_DIG_Z2_LSB  = 0x68
BMM_DIG_Z2_MSB  = 0x69
BMM_DIG_Z1_LSB  = 0x6A
BMM_DIG_Z1_MSB  = 0x6B
BMM_DIG_XYZ1_LSB = 0x6C
BMM_DIG_XYZ1_MSB = 0x6D
BMM_DIG_Z3_LSB  = 0x6E
BMM_DIG_Z3_MSB  = 0x6F
BMM_DIG_XY2     = 0x70
BMM_DIG_XY1     = 0x71
BMM_ID_EXPECTED = 0x32
BMM_QUANTUM_MG  = 1.0 / 1.6
MG_TO_UT        = 0.1

bus = smbus2.SMBus(1)

def write_byte(addr, reg, val):
    try:
        bus.write_byte_data(addr, reg, val)
    except Exception as e:
        print(f"Ошибка записи {addr:#x}:{reg:#x} = {val:#x}: {e}")

def read_byte(addr, reg):
    try:
        return bus.read_byte_data(addr, reg)
    except:
        return None

def read_block(addr, reg, length):
    try:
        return bus.read_i2c_block_data(addr, reg, length)
    except Exception as e:
        print(f"Ошибка чтения блока {addr:#x}:{reg:#x} len={length}: {e}")
        return None

def twos_complement(val, bits):
    if val & (1 << (bits - 1)):
        val -= (1 << bits)
    return val

# ======================== КЛАССЫ ДАТЧИКОВ ========================
class Accel:
    def __init__(self):
        self.ax = self.ay = self.az = 0.0
        self.temp = 0.0

    def begin(self):
        chip_id = read_byte(BMA_ADDR, BMA_CHIP_ID)
        if chip_id != BMA_ID_EXPECTED:
            raise RuntimeError(f"Акселерометр не найден (ID={chip_id:#x})")
        write_byte(BMA_ADDR, BMA_RESET, 0xB6)
        time.sleep(0.1)
        write_byte(BMA_ADDR, BMA_PMU_LPW, 0x00)
        time.sleep(0.01)
        write_byte(BMA_ADDR, BMA_ACC_RANGE, BMA_RANGE_2G)
        write_byte(BMA_ADDR, BMA_ACC_BW, BMA_BW_125HZ)
        write_byte(BMA_ADDR, BMA_D_HBW, 0x00)
        print("Акселерометр инициализирован.")

    def read(self):
        data = read_block(BMA_ADDR, BMA_ACC_DATA, 7)
        if not data:
            return False
        x_raw = twos_complement((data[1] << 8) | (data[0] & 0xF0), 16) >> 4
        y_raw = twos_complement((data[3] << 8) | (data[2] & 0xF0), 16) >> 4
        z_raw = twos_complement((data[5] << 8) | (data[4] & 0xF0), 16) >> 4
        scale = 2.0 / 2048.0
        self.ax = x_raw * scale
        self.ay = y_raw * scale
        self.az = z_raw * scale
        self.temp = twos_complement(data[6], 8) * 0.5 + 23.0
        return True

    def get_angles(self):
        pitch = math.atan2(self.ax, math.sqrt(self.ay*self.ay + self.az*self.az))
        roll  = math.atan2(self.ay, math.sqrt(self.ax*self.ax + self.az*self.az))
        return math.degrees(roll), math.degrees(pitch)

class Gyro:
    def __init__(self):
        self.gx = self.gy = self.gz = 0.0
        self.offset_x = self.offset_y = self.offset_z = 0.0

    def begin(self):
        chip_id = read_byte(BMG_ADDR, BMG_CHIP_ID)
        if chip_id != BMG_ID_EXPECTED:
            raise RuntimeError(f"Гироскоп не найден (ID={chip_id:#x})")
        write_byte(BMG_ADDR, BMG_RESET, 0xB6)
        time.sleep(0.1)
        write_byte(BMG_ADDR, BMG_LPM1, 0x00)
        write_byte(BMG_ADDR, BMG_RANGE, BMG_RANGE_125DPS)
        write_byte(BMG_ADDR, BMG_BW, BMG_BW_23HZ)
        write_byte(BMG_ADDR, BMG_D_HBW, 0x00)
        print("Гироскоп инициализирован.")

    def read(self):
        data = read_block(BMG_ADDR, BMG_DATA, 6)
        if not data:
            return False
        x_raw = twos_complement((data[1] << 8) | data[0], 16)
        y_raw = twos_complement((data[3] << 8) | data[2], 16)
        z_raw = twos_complement((data[5] << 8) | data[4], 16)
        self.gx = x_raw * BMG_SCALE_125
        self.gy = y_raw * BMG_SCALE_125
        self.gz = z_raw * BMG_SCALE_125
        return True

    def read_calibrated(self):
        if not self.read():
            return False
        self.gx -= self.offset_x
        self.gy -= self.offset_y
        self.gz -= self.offset_z
        return True

    def calibrate(self, samples=100):
        print("Калибровка гироскопа (не двигайте датчик)...")
        sx = sy = sz = 0.0
        count = 0
        for _ in range(samples):
            if self.read():
                sx += self.gx
                sy += self.gy
                sz += self.gz
                count += 1
            time.sleep(0.01)
        if count > 0:
            self.offset_x = sx / count
            self.offset_y = sy / count
            self.offset_z = sz / count
            print(f"Смещения гироскопа: X={self.offset_x:.3f}, Y={self.offset_y:.3f}, Z={self.offset_z:.3f} °/с")
        else:
            print("Не удалось получить данные для калибровки гироскопа.")

class Mag:
    def __init__(self):
        self.mx = self.my = self.mz = 0.0
        self.temp = 0.0
        self.dig_x1 = self.dig_y1 = self.dig_x2 = self.dig_y2 = 0
        self.dig_z1 = self.dig_z2 = self.dig_z3 = self.dig_z4 = 0
        self.dig_xy1 = self.dig_xy2 = 0
        self.dig_xyz1 = 0
        self.bias = [0.0, 0.0, 0.0]

    def begin(self):
        write_byte(BMM_ADDR, BMM_POWER, 0x01)
        time.sleep(0.1)
        chip_id = read_byte(BMM_ADDR, BMM_CHIP_ID)
        if chip_id != BMM_ID_EXPECTED:
            raise RuntimeError(f"Магнитометр не найден (ID={chip_id:#x})")
        write_byte(BMM_ADDR, BMM_CTRL1, 0x00)
        write_byte(BMM_ADDR, BMM_REP_XY, (9-1)//2)
        write_byte(BMM_ADDR, BMM_REP_Z, 15-1)

        self.dig_x1 = read_byte(BMM_ADDR, BMM_DIG_X1)
        self.dig_y1 = read_byte(BMM_ADDR, BMM_DIG_Y1)
        self.dig_x2 = read_byte(BMM_ADDR, BMM_DIG_X2)
        self.dig_y2 = read_byte(BMM_ADDR, BMM_DIG_Y2)
        self.dig_xy1 = read_byte(BMM_ADDR, BMM_DIG_XY1)
        self.dig_xy2 = read_byte(BMM_ADDR, BMM_DIG_XY2)

        data = read_block(BMM_ADDR, BMM_DIG_Z1_LSB, 2)
        if data:
            self.dig_z1 = (data[1] << 8) | data[0]
        data = read_block(BMM_ADDR, BMM_DIG_Z2_LSB, 2)
        if data:
            self.dig_z2 = twos_complement((data[1] << 8) | data[0], 16)
        data = read_block(BMM_ADDR, BMM_DIG_Z3_LSB, 2)
        if data:
            self.dig_z3 = twos_complement((data[1] << 8) | data[0], 16)
        data = read_block(BMM_ADDR, BMM_DIG_Z4_LSB, 2)
        if data:
            self.dig_z4 = twos_complement((data[1] << 8) | data[0], 16)
        data = read_block(BMM_ADDR, BMM_DIG_XYZ1_LSB, 2)
        if data:
            self.dig_xyz1 = (data[1] << 8) | data[0]
        print("Магнитометр инициализирован.")

    def read_raw_compensated(self):
        data = read_block(BMM_ADDR, BMM_DATA, 8)
        if not data:
            return None
        # if not (data[6] & 0x01):
        #     return None

        x_raw = (data[1] << 8) | data[0]
        y_raw = (data[3] << 8) | data[2]
        z_raw = (data[5] << 8) | data[4]
        rhall = (data[7] << 8) | data[6]

        rhall_val = rhall if rhall != 0 else self.dig_xyz1
        t = (self.dig_xyz1 << 14) // rhall_val - 0x4000
        t = twos_complement(t & 0xFFFF, 16)

        # X
        x_comp = self.dig_x1 << 3
        part1 = (self.dig_xy2 * ((t * t) >> 7)) + (t * (self.dig_xy1 << 7))
        part2 = (part1 >> 9) + 0x100000
        part3 = part2 * (self.dig_x2 + 0xA0)
        part4 = part3 >> 12
        x_comp += ( ((x_raw >> 3) * part4) >> 13 )
        x_comp = twos_complement(x_comp & 0xFFFF, 16)

        # Y
        y_comp = self.dig_y1 << 3
        part1 = (self.dig_xy2 * ((t * t) >> 7)) + (t * (self.dig_xy1 << 7))
        part2 = (part1 >> 9) + 0x100000
        part3 = part2 * (self.dig_y2 + 0xA0)
        part4 = part3 >> 12
        y_comp += ( ((y_raw >> 3) * part4) >> 13 )
        y_comp = twos_complement(y_comp & 0xFFFF, 16)

        # Z
        z_part = (z_raw >> 1) - self.dig_z4
        numerator = (z_part << 15) - ((self.dig_z3 * (rhall - self.dig_xyz1)) >> 2)
        denominator = self.dig_z2 + ( ((self.dig_z1 * (rhall << 1)) + (1 << 15)) >> 16 )
        z_comp = numerator // denominator
        z_comp = twos_complement(z_comp & 0xFFFF, 16)

        x_mg = x_comp * BMM_QUANTUM_MG
        y_mg = y_comp * BMM_QUANTUM_MG
        z_mg = z_comp * BMM_QUANTUM_MG
        return (x_mg, y_mg, z_mg)

    def calibrate(self, duration=15):
        print(f"\nКалибровка магнитометра. Вращайте датчик во всех направлениях в течение {duration} секунд...")
        max_x = max_y = max_z = -1e9
        min_x = min_y = min_z = 1e9
        start = time.time()
        count = 0
        while time.time() - start < duration:
            comp = self.read_raw_compensated()
            if comp:
                x, y, z = comp
                if x > max_x: max_x = x
                if y > max_y: max_y = y
                if z > max_z: max_z = z
                if x < min_x: min_x = x
                if y < min_y: min_y = y
                if z < min_z: min_z = z
                count += 1
            time.sleep(0.05)
        if count == 0:
            print("Не удалось получить данные для калибровки магнитометра.")
            return
        self.bias[0] = (max_x + min_x) / 2.0
        self.bias[1] = (max_y + min_y) / 2.0
        self.bias[2] = (max_z + min_z) / 2.0
        print(f"Калибровка магнитометра завершена. Смещения (мГс): X={self.bias[0]:.1f}, Y={self.bias[1]:.1f}, Z={self.bias[2]:.1f}")

# ======================== ФИЛЬТР МАДЖВИКА (СВОЯ РЕАЛИЗАЦИЯ) ========================
class MadgwickFilter:
    def __init__(self, beta=1.0):
        self.beta = beta
        self.q = [1.0, 0.0, 0.0, 0.0]  # кватернион (w, x, y, z)

    def update(self, ax, ay, az, gx, gy, gz, mx, my, mz, dt):
        # Нормализация акселерометра и магнитометра
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0.0: return
        ax /= norm; ay /= norm; az /= norm

        norm = math.sqrt(mx*mx + my*my + mz*mz)
        if norm == 0.0: return
        mx /= norm; my /= norm; mz /= norm

        q0, q1, q2, q3 = self.q

        # Вспомогательные переменные
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3

        # Градиент спуска по акселерометру и магнитометру
        # (стандартная реализация MadgwickAHRS)
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay

        # Нормализация градиента
        norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm > 0.0:
            s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm
        else:
            s0 = s1 = s2 = s3 = 0.0

        # Производная кватерниона от гироскопа
        qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz)
        qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy)
        qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx)
        qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx)

        # Интегрирование с коррекцией
        self.q[0] += (qDot1 - self.beta * s0) * dt
        self.q[1] += (qDot2 - self.beta * s1) * dt
        self.q[2] += (qDot3 - self.beta * s2) * dt
        self.q[3] += (qDot4 - self.beta * s3) * dt

        # Нормализация кватерниона
        norm = math.sqrt(self.q[0]**2 + self.q[1]**2 + self.q[2]**2 + self.q[3]**2)
        if norm > 0.0:
            self.q[0] /= norm
            self.q[1] /= norm
            self.q[2] /= norm
            self.q[3] /= norm

    def get_euler(self):
        q0, q1, q2, q3 = self.q
        roll = math.atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2))
        pitch = math.asin(2.0*(q0*q2 - q3*q1))
        yaw = math.atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

# ======================== ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ ========================
def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz

# ======================== ОСНОВНАЯ ПРОГРАММА ========================
if __name__ == "__main__":
    print("Инициализация датчиков BMX055...")
    accel = Accel()
    gyro = Gyro()
    mag = Mag()

    try:
        accel.begin()
        gyro.begin()
        mag.begin()
    except Exception as e:
        print(f"Ошибка инициализации: {e}")
        bus.close()
        exit(1)

    gyro.calibrate(samples=100)

    # Фиксированные смещения магнитометра (из предыдущих калибровок)
    FIXED_MAG_BIAS_X = 1235.0
    FIXED_MAG_BIAS_Y = 1220.0
    FIXED_MAG_BIAS_Z = -2200.0
    mag.bias = [FIXED_MAG_BIAS_X, FIXED_MAG_BIAS_Y, FIXED_MAG_BIAS_Z]
    print(f"Используются фиксированные смещения магнитометра (мГс): X={FIXED_MAG_BIAS_X:.1f}, Y={FIXED_MAG_BIAS_Y:.1f}, Z={FIXED_MAG_BIAS_Z:.1f}")

    # Определение начального направления (для относительного угла)
    print("\nОпределение начального направления (не двигайте датчик)...")
    time.sleep(1)
    yaw_samples = []
    for _ in range(10):
        raw = mag.read_raw_compensated()
        if raw:
            x_mg, y_mg, z_mg = raw
            x_mg -= mag.bias[0]
            y_mg -= mag.bias[1]
            z_mg -= mag.bias[2]
            # Временно используем стандартную перестановку для получения Yaw_abs
            mx = -y_mg * MG_TO_UT
            my =  x_mg * MG_TO_UT
            yaw_rad = math.atan2(my, mx)
            yaw_deg = math.degrees(yaw_rad)
            if yaw_deg < 0:
                yaw_deg += 360
            yaw_samples.append(yaw_deg)
        time.sleep(0.1)

    if yaw_samples:
        yaw_offset = sum(yaw_samples) / len(yaw_samples)
        print(f"Начальное направление (абсолютный курс): {yaw_offset:.1f}°. Оно будет считаться нулём.")
    else:
        yaw_offset = 0.0
        print("Не удалось получить начальное направление, используется 0°.")

    # Создаём фильтр
    filter = MadgwickFilter(beta=BETA)

    # Инициализация кватерниона
    if USE_INIT_QUAT:
        print("Инициализация кватерниона по первым измерениям...")
        # Получаем первые данные
        for _ in range(5):
            accel.read()
            gyro.read_calibrated()
            raw = mag.read_raw_compensated()
            time.sleep(0.05)

        ax, ay, az = accel.ax, accel.ay, accel.az
        roll0 = math.atan2(ay, az)
        pitch0 = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        if raw:
            x_mg, y_mg, z_mg = raw
            x_mg -= mag.bias[0]
            y_mg -= mag.bias[1]
            z_mg -= mag.bias[2]
            mx0 = -y_mg * MG_TO_UT
            my0 =  x_mg * MG_TO_UT
            yaw0 = math.atan2(my0, mx0)
        else:
            yaw0 = 0.0

        qw, qx, qy, qz = euler_to_quat(roll0, pitch0, yaw0)
        filter.q = [qw, qx, qy, qz]
        print(f"Начальный кватернион: ({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})")

    print("\n--- Начинаем вывод данных (нажмите Ctrl+C для выхода) ---\n")
    print("Raw (mG)       – сырые данные магнитометра (после температурной компенсации, БЕЗ вычитания bias и БЕЗ перестановки осей)")
    print("Corr (µT)      – данные после вычитания bias и перестановки осей (X=-Y_raw, Y=+X_raw, Z=Z_raw)")
    print("Yaw_abs        – абсолютный курс из магнитометра (0–360°)")
    print("Yaw_rel        – относительный курс из магнитометра (0–360°, 0° = начальное направление)")
    print("Filt (roll/pitch/yaw) – углы после фильтра Маджвика")
    print("-" * 80)

    last_time = time.time()
    initial_fyaw = None  # для запоминания начального yaw фильтра

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # Акселерометр
            if accel.read():
                ax, ay, az = accel.ax, accel.ay, accel.az
                aroll, apitch = accel.get_angles()
            else:
                ax = ay = az = 0.0
                aroll = apitch = 0.0

            # Гироскоп (калиброванный)
            if gyro.read_calibrated():
                gx, gy, gz = gyro.gx, gyro.gy, gyro.gz
            else:
                gx = gy = gz = 0.0

            # Магнитометр
            raw_mag = mag.read_raw_compensated()
            if raw_mag:
                raw_x, raw_y, raw_z = raw_mag
                x_mg = raw_x - mag.bias[0]
                y_mg = raw_y - mag.bias[1]
                z_mg = raw_z - mag.bias[2]

                if MAG_OPTION == 1:
                    mx = -y_mg * MG_TO_UT
                    my =  x_mg * MG_TO_UT
                    mz =  z_mg * MG_TO_UT
                elif MAG_OPTION == 2:
                    mx = x_mg * MG_TO_UT
                    my = y_mg * MG_TO_UT
                    mz = z_mg * MG_TO_UT
                elif MAG_OPTION == 3:
                    mx = y_mg * MG_TO_UT
                    my = -x_mg * MG_TO_UT
                    mz = z_mg * MG_TO_UT
                else:
                    mx = -y_mg * MG_TO_UT
                    my =  x_mg * MG_TO_UT
                    mz =  z_mg * MG_TO_UT

                yaw_abs = math.degrees(math.atan2(my, mx))
                if yaw_abs < 0:
                    yaw_abs += 360
                yaw_rel = (yaw_abs - yaw_offset) % 360
            else:
                raw_x = raw_y = raw_z = 0.0
                mx = my = mz = 0.0
                yaw_abs = 0.0
                yaw_rel = 0.0

            # Подготовка данных для фильтра
            gx_rad = math.radians(gx) * GYRO_SIGN
            gy_rad = math.radians(gy) * GYRO_SIGN
            gz_rad = math.radians(gz) * GYRO_SIGN

            # ----------------- ОТЛАДКА: вывод значений перед фильтром -----------------
            print(f"DEBUG: mx={mx:7.2f}, my={my:7.2f}, mz={mz:7.2f}")
            # --------------------------------------------------------------------------

            # Обновление фильтра
            filter.update(ax, ay, az, gx_rad, gy_rad, gz_rad, mx, my, mz, dt)
            froll, fpitch, fyaw = filter.get_euler()
            if fyaw < 0:
                fyaw += 360

            # Запоминаем начальный yaw (первый достоверный)
            if initial_fyaw is None and abs(fyaw) > 0.1:
                initial_fyaw = fyaw
                print(f"Начальный угол фильтра: {initial_fyaw:.1f}° (будет считаться нулём)")

            # Относительный yaw фильтра (по часовой стрелке положительный)
            if initial_fyaw is not None:
                f_yaw_rel = (initial_fyaw - fyaw) % 360
            else:
                f_yaw_rel = 0.0

            # Вывод
            print(f"Acc(g): X={ax:6.3f} Y={ay:6.3f} Z={az:6.3f}  |  Gyr(°/s): X={gx:7.2f} Y={gy:7.2f} Z={gz:7.2f}")
            print(f"Raw (mG): X={raw_x:8.1f} Y={raw_y:8.1f} Z={raw_z:8.1f}  |  Corr (µT): X={mx:7.2f} Y={my:7.2f} Z={mz:7.2f}  |  Yaw_abs={yaw_abs:6.1f}° Yaw_rel={yaw_rel:6.1f}°")
            print(f"Filtered (deg): Roll={froll:6.2f} Pitch={fpitch:6.2f} Yaw={fyaw:6.1f}°  Rel Yaw={f_yaw_rel:6.1f}°")
            print("-" * 80)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nВыход по запросу пользователя.")
    finally:
        bus.close()
