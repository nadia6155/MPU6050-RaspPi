from smbus2 import SMBus
import time
from math import atan, pow, pi, sqrt

# set variables for mpu6050 registers 
MPU = 0x68
PWR_MGMT_1 = 0x6B # used to wake up sensor (turn off sleep)
ACCEL_XOUT_H = 0x3B # first accel register 
GYRO_XOUT_H = 0x43 # first gyro register

bus = SMBus(1)

# wake up sensor by sending bytes to power mgmt register
bus.write_byte_data(MPU, PWR_MGMT_1, 0)

# initialise global variables 
GyroX_angle = 0.0
GyroY_angle = 0.0
yaw = 0.0
current_time = time.time() * 1000

# func to read signed 16-bit value from register
def read_word_2c(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    return val

def accel_read():

    # read 2 bytes from register
    AccX_raw = read_word_2c(MPU, ACCEL_XOUT_H)
    AccY_raw = read_word_2c(MPU, ACCEL_XOUT_H + 2)
    AccZ_raw = read_word_2c(MPU, ACCEL_XOUT_H + 4)

    # turn raw data into g-force unit of measurement 
    AccX = AccX_raw / 16384.0
    AccY = AccY_raw / 16384.0
    AccZ = AccZ_raw / 16384.0

    AccX_angle = atan(AccY) / (sqrt(pow(AccX, 2) + pow(AccZ, 2)) * 180 / pi)

    AccY_angle = atan(-1 * AccX) / (sqrt(pow(AccY, 2) + pow(AccZ, 2)) * 180 / pi)

    return AccX_angle, AccY_angle

def millis():
    milliseconds = int(time() * 1000)
    return milliseconds

def gyro_read():
    
    global current_time, GyroX_angle, GyroY_angle

    previous_time = current_time
    current_time = time.time() * 1000
    elapsed_time = (current_time - previous_time) / 1000

    GyroX_raw = read_word_2c(MPU, GYRO_XOUT_H)
    GyroY_raw = read_word_2c(MPU, GYRO_XOUT_H + 2)
    GyroZ_raw = read_word_2c(MPU, GYRO_XOUT_H + 4)

    GyroX = GyroX_raw / 131.0
    GyroY = GyroY_raw / 131.0
    GyroZ = GyroZ_raw / 131.0

    GyroX_angle = GyroX_angle + GyroX * elapsed_time
    GyroY_angle = GyroY_angle + GyroY * elapsed_time 

    return GyroZ, GyroX_angle, GyroY_angle, elapsed_time

def calculate_axis_motion(GyroZ, elapsed_time, GyroX_angle, AccX_angle, GyroY_angle, AccY_angle):
    global yaw

    yaw = yaw + GyroZ * elapsed_time

    roll = 0.96 * GyroX_angle + 0.04 * AccX_angle

    pitch = 0.96 * GyroY_angle + 0.04 * AccY_angle

    return(pitch, roll, yaw)

def main():
    
    AccX_angle, AccY_angle = accel_read()
    GyroZ, GyroX_angle, GyroY_angle, elapsed_time = gyro_read()
    pitch, roll, yaw = calculate_axis_motion(GyroZ, elapsed_time, GyroX_angle, AccX_angle, GyroY_angle, AccY_angle)

    print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

if __name__=="__main__":
    while True:
        main()
        time.sleep(0.1) # 10 readings per second
