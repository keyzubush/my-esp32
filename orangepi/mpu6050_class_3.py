#!/usr/bin/env python

from smbus2 import SMBus
import math
import time
from multiprocessing import Process, Manager
from ctypes import c_float

class MPU6050:
    def __init__(self, bus_num=3, address=0x68, sample_rate=0.1):
        """
        Initialize MPU6050 sensor with I2C communication.
        
        Args:
            bus_num (int): I2C bus number (default: 3)
            address (int): I2C slave address (default: 0x68)
            sample_rate (float): Sampling interval in seconds (default: 0.1)
        """
        # MPU6050 Register addresses
        self.PWR_MGMT_1 = 0x6B
        self.PWR_MGMT_2 = 0x6C
        self.WHO_AM_I = 0x75
        self.GYRO_X = 0x43
        self.GYRO_Y = 0x45
        self.GYRO_Z = 0x47
        self.ACCL_X = 0x3B
        self.ACCL_Y = 0x3D
        self.ACCL_Z = 0x3F
        
        self.bus_num = bus_num
        self.address = address
        self.sample_rate = sample_rate
        
        # Create shared dictionary for communication between processes
        self.manager = Manager()
        self.shared_data = self.manager.dict()
        
        # Initialize shared data structure
        self.reset()
        
        # Process control
        self.process = None
        self.running = False
        
    def _initialize_sensor(self):
        """Initialize the MPU6050 sensor."""
        bus = SMBus(self.bus_num)
        bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
        return bus
    
    def _read_word(self, bus, addr):
        """Read a 16-bit word from the specified address."""
        h = bus.read_byte_data(self.address, addr)
        l = bus.read_byte_data(self.address, addr+1)
        val = (h << 8) + l
        return val
    
    def _read_word_i2c(self, bus, addr):
        """Read a 16-bit word with proper sign handling."""
        val = self._read_word(bus, addr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
    
    def _dist(self, x, y):
        """Calculate the distance of x and y from origin."""
        return math.sqrt((x*x) + (y*y))
    
    def _get_x_rotation(self, x, y, z):
        """Calculate X-axis rotation angle in degrees."""
        rad = math.atan2(y, self._dist(x, z))
        return math.degrees(rad)
    
    def _get_y_rotation(self, x, y, z):
        """Calculate Y-axis rotation angle in degrees."""
        rad = math.atan2(x, self._dist(y, z))
        return -math.degrees(rad)
    
    def _sensor_loop(self, shared_data):
        """Main sensor reading loop to run in separate process."""
        bus = self._initialize_sensor()
        
        # Initialize calibration offsets
        gyro_offsets = shared_data['gyro_offsets']
        accel_offsets = shared_data['accel_offsets']
        
        # Initialize integrated values
        integrated_gyro_x = shared_data['integrated_gyro_x']
        integrated_gyro_y = shared_data['integrated_gyro_y']
        integrated_gyro_z = shared_data['integrated_gyro_z']
        
        # Initialize distance calculation variables
        velocity_x = shared_data['velocity_x']
        velocity_y = shared_data['velocity_y']
        velocity_z = shared_data['velocity_z']
        distance = shared_data['distance']
        
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Read gyro data (degrees per second) with offsets applied
            GYR_X = self._read_word_i2c(bus, self.GYRO_X) / 131.0 - gyro_offsets['x']
            GYR_Y = self._read_word_i2c(bus, self.GYRO_Y) / 131.0 - gyro_offsets['y']
            GYR_Z = self._read_word_i2c(bus, self.GYRO_Z) / 131.0 - gyro_offsets['z']
           
            # Integrate gyro values (degrees)
            integrated_gyro_x.value += GYR_X * dt
            integrated_gyro_y.value += GYR_Y * dt
            integrated_gyro_z.value += GYR_Z * dt
            
            # Read accelerometer data (g) with offsets applied
            ACC_X = self._read_word_i2c(bus, self.ACCL_X) / 16384.0 - accel_offsets['x']
            ACC_Y = self._read_word_i2c(bus, self.ACCL_Y) / 16384.0 - accel_offsets['y']
            ACC_Z = self._read_word_i2c(bus, self.ACCL_Z) / 16384.0 - accel_offsets['z']
            
            # Apply gravity compensation to accelerometer data
            ACC_Z = ACC_Z - 1.0  # Remove 1g from Z-axis (assuming sensor is flat)
            
            # Calculate rotations from accelerometer
            rot_x = self._get_x_rotation(ACC_X, ACC_Y, ACC_Z)
            rot_y = self._get_y_rotation(ACC_X, ACC_Y, ACC_Z)

            # Calculate velocity and distance (convert g to m/s^2 by multiplying by 9.81)
            velocity_x.value += ACC_X * 9.81 * dt
            velocity_y.value += ACC_Y * 9.81 * dt
            velocity_z.value += ACC_Z * 9.81 * dt

            if abs(ACC_X) + abs(ACC_Y) + abs(ACC_Z) < 0.035:
                velocity_x.value = 0.0
                velocity_y.value = 0.0
                velocity_z.value = 0.0
            
            # Calculate distance in 2D plane (ignoring Z-axis for most applications)
            distance_2d = math.sqrt((velocity_x.value * dt)**2 + (velocity_y.value * dt)**2)
            distance.value += distance_2d
            
            # Update shared dictionary
            shared_data.update({
                'gyro_x': GYR_X,
                'gyro_y': GYR_Y,
                'gyro_z': GYR_Z,
                'accel_x': ACC_X,
                'accel_y': ACC_Y,
                'accel_z': ACC_Z,
                'rotation_x': rot_x,
                'rotation_y': rot_y,
                'velocity_x': velocity_x.value,
                'velocity_y': velocity_y.value,
                'velocity_z': velocity_z.value,
                'distance': distance.value,
                'timestamp': current_time
            })
            
            time.sleep(self.sample_rate)
    
    def calibrate_gyro(self, samples=100):
        """
        Calibrate the gyroscope by calculating offsets.
        Sensor should be stationary during calibration.
        
        Args:
            samples (int): Number of samples to average for calibration (default: 100)
        """
        if self.running:
            raise RuntimeError("Cannot calibrate while sensor is running")
            
        bus = self._initialize_sensor()
        
        # Collect samples
        sum_x, sum_y, sum_z = 0, 0, 0
        for _ in range(samples):
            sum_x += self._read_word_i2c(bus, self.GYRO_X) / 131.0
            sum_y += self._read_word_i2c(bus, self.GYRO_Y) / 131.0
            sum_z += self._read_word_i2c(bus, self.GYRO_Z) / 131.0
            time.sleep(0.01)
        
        # Calculate average offsets
        self.shared_data['gyro_offsets'] = {
            'x': sum_x / samples,
            'y': sum_y / samples,
            'z': sum_z / samples
        }
        print(f"Gyro calibration complete. Offsets: X={sum_x/samples:.2f}, Y={sum_y/samples:.2f}, Z={sum_z/samples:.2f}")
    
    def calibrate_accel(self, samples=100):
        """
        Calibrate the accelerometer by calculating offsets.
        Sensor should be placed flat on a level surface during calibration.
        
        Args:
            samples (int): Number of samples to average for calibration (default: 100)
        """
        if self.running:
            raise RuntimeError("Cannot calibrate while sensor is running")
            
        bus = self._initialize_sensor()
        
        # Collect samples
        sum_x, sum_y, sum_z = 0, 0, 0
        for _ in range(samples):
            sum_x += self._read_word_i2c(bus, self.ACCL_X) / 16384.0
            sum_y += self._read_word_i2c(bus, self.ACCL_Y) / 16384.0
            sum_z += self._read_word_i2c(bus, self.ACCL_Z) / 16384.0
            time.sleep(0.01)
        
        # Calculate average offsets
        # Z-axis should measure 1g when flat (subtract 1 from average)
        self.shared_data['accel_offsets'] = {
            'x': sum_x / samples,
            'y': sum_y / samples,
            'z': (sum_z / samples) - 1.0  # Remove gravity from Z-axis
        }
        print(f"Accel calibration complete. Offsets: X={sum_x/samples:.2f}, Y={sum_y/samples:.2f}, Z={(sum_z/samples)-1.0:.2f}")
    
    def calibrate(self, samples=100):
        """
        Convenience method to calibrate both gyro and accelerometer.
        """
        print("Starting gyroscope calibration...")
        self.calibrate_gyro(samples)
        print("Starting accelerometer calibration...")
        self.calibrate_accel(samples)
        print("Calibration complete!")
    
    def reset(self):
        """Reset the integrated values to zero."""
        # Using ctypes values for atomic operations in shared memory
        self.shared_data.update({
            'gyro_x': 0.0,
            'gyro_y': 0.0,
            'gyro_z': 0.0,
            'accel_x': 0.0,
            'accel_y': 0.0,
            'accel_z': 0.0,
            'rotation_x': 0.0,
            'rotation_y': 0.0,
            'velocity_x': self.manager.Value(c_float, 0.0),
            'velocity_y': self.manager.Value(c_float, 0.0),
            'velocity_z': self.manager.Value(c_float, 0.0),
            'distance': self.manager.Value(c_float, 0.0),
            'timestamp': 0.0,
            'gyro_offsets': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'accel_offsets': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'integrated_gyro_x': self.manager.Value(c_float, 0.0),
            'integrated_gyro_y': self.manager.Value(c_float, 0.0),
            'integrated_gyro_z': self.manager.Value(c_float, 0.0)
        })
    
    def start(self):
        """Start the sensor reading process."""
        if not self.running:
            self.running = True
            self.process = Process(target=self._sensor_loop, args=(self.shared_data,))
            self.process.start()
    
    def stop(self):
        """Stop the sensor reading process."""
        if self.running:
            self.running = False
            self.process.join()
            self.process = None
    
    def get_data(self):
        """Get the current sensor data from shared memory."""
        data = dict(self.shared_data)
        # Create a new dictionary with all values properly extracted
        return {
            'gyro_x': data['gyro_x'],
            'gyro_y': data['gyro_y'],
            'gyro_z': data['gyro_z'],
            'accel_x': data['accel_x'],
            'accel_y': data['accel_y'],
            'accel_z': data['accel_z'],
            'rotation_x': data['rotation_x'],
            'rotation_y': data['rotation_y'],
            'velocity_x': data['velocity_x'].value if hasattr(data['velocity_x'], 'value') else data['velocity_x'],
            'velocity_y': data['velocity_y'].value if hasattr(data['velocity_y'], 'value') else data['velocity_y'],
            'velocity_z': data['velocity_z'].value if hasattr(data['velocity_z'], 'value') else data['velocity_z'],
            'distance': data['distance'].value if hasattr(data['distance'], 'value') else data['distance'],
            'timestamp': data['timestamp'],
            'gyro_offsets': data['gyro_offsets'],
            'accel_offsets': data['accel_offsets'],
            'integrated_gyro_x': data['integrated_gyro_x'].value if hasattr(data['integrated_gyro_x'], 'value') else data['integrated_gyro_x'],
            'integrated_gyro_y': data['integrated_gyro_y'].value if hasattr(data['integrated_gyro_y'], 'value') else data['integrated_gyro_y'],
            'integrated_gyro_z': data['integrated_gyro_z'].value if hasattr(data['integrated_gyro_z'], 'value') else data['integrated_gyro_z']
        }
    
    def __del__(self):
        """Ensure the process is stopped when the object is deleted."""
        self.stop()

if __name__ == "__main__":
    # Create MPU6050 instance
    mpu = MPU6050()
    
    # Calibrate both gyroscope and accelerometer
    print("Calibrating sensors... (keep sensor stationary and flat)")
    mpu.calibrate()
    
    # Start the sensor reading process
    mpu.start()
    
    try:
        while True:
            # Get current sensor data
            data = mpu.get_data()
            
            # Print some values
            print("\n--- Sensor Data ---")
            print(f"Rotation: X={data['rotation_x']:.2f}°, Y={data['rotation_y']:.2f}°")
            print(f"Gyro: X={data['gyro_x']:.2f}°/s, Y={data['gyro_y']:.2f}°/s, Z={data['gyro_z']:.2f}°/s")
            print(f"Accel: X={data['accel_x']:.2f}g, Y={data['accel_y']:.2f}g, Z={data['accel_z']:.2f}g")
            print(f"Velocity: X={data['velocity_x']:.2f}m/s, Y={data['velocity_y']:.2f}m/s")
            print(f"Distance: {data['distance']:.2f}m")
            print(f"Integrated Gyro Z: {data['integrated_gyro_z']:.2f}°")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("Stopping...")
        mpu.stop()
