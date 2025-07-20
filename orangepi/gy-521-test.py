import time
from smbus2 import SMBus, i2c_msg

# MPU-6050 I2C Address
MPU6050_ADDR = 0x68

# Register addresses (from MPU-6050 datasheet)
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Initialize I2C (use /dev/i2c-2 if SDA.2/SCL.2)
I2C_BUS = 3

def mpu6050_init(bus):
    # Wake up MPU-6050 (disable sleep mode)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)

def read_sensor_data(bus):
    # Read 14 bytes (accel X/Y/Z, temp, gyro X/Y/Z)
    msg = i2c_msg.read(MPU6050_ADDR, 14)
    bus.i2c_rdwr(msg)
    data = list(msg)

    # Convert raw bytes to 16-bit integers
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]
    temp = (data[6] << 8) | data[7]
    gyro_x = (data[8] << 8) | data[9]
    gyro_y = (data[10] << 8) | data[11]
    gyro_z = (data[12] << 8) | data[13]

    # Convert to human-readable values
    accel_scale = 16384.0  # for ±2g range
    gyro_scale = 131.0     # for ±250°/s range

    accel_x = accel_x / accel_scale
    accel_y = accel_y / accel_scale
    accel_z = accel_z / accel_scale
    temp = temp / 340.0 + 36.53  # Temperature in °C
    gyro_x = gyro_x / gyro_scale
    gyro_y = gyro_y / gyro_scale
    gyro_z = gyro_z / gyro_scale

    return (accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z)

def main():
    try:
        # Initialize I2C bus
        with SMBus(I2C_BUS) as bus:
            mpu6050_init(bus)
            print("MPU-6050 Ready. Reading data...")

            while True:
                accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z = read_sensor_data(bus)
                print(f"Accel (g): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}")
                print(f"Gyro (°/s): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
                print(f"Temp (°C): {temp:.2f}")
                print("-----------------------")
                time.sleep(0.2)  # Adjust delay for read frequency

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
