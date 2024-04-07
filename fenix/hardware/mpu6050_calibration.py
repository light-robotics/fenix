import math
import time
from mpu6050 import mpu6050


accel_bias = {'x': 0, 'y': 0, 'z': 0}  # Update with your accelerometer bias values
gyro_bias = {'x': 0, 'y': 0, 'z': 0}

def read_sensor_data(sensor):
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    return accel_data, gyro_data

def complementary_filter(accel_data, gyro_data, accel_bias, gyro_bias, dt, alpha=0.98):
    # Subtract biases from accelerometer and gyroscope data
    accel_data = {axis: accel_data[axis] - accel_bias[axis] for axis in accel_data}
    gyro_data = {axis: gyro_data[axis] - gyro_bias[axis] for axis in gyro_data}
    
    # Convert accelerometer data to pitch and roll angles
    pitch_acc = math.atan2(accel_data['y'], math.sqrt(accel_data['x']**2 + accel_data['z']**2))
    roll_acc = math.atan2(-accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2))

    # Convert gyroscope data to radians per second
    gyro_data_rad_per_sec = {
        'x': math.radians(gyro_data['x']),
        'y': math.radians(gyro_data['y']),
        'z': math.radians(gyro_data['z'])
    }

    # Calculate the change in pitch and roll using gyroscope data
    pitch_gyro = gyro_data_rad_per_sec['x'] * dt  # Change in pitch
    roll_gyro = gyro_data_rad_per_sec['y'] * dt   # Change in roll

    # Combine accelerometer and gyroscope data for pitch and roll using complementary filter
    pitch = alpha * (pitch_acc + pitch_gyro) + (1 - alpha) * pitch_acc
    roll = alpha * (roll_acc + roll_gyro) + (1 - alpha) * roll_acc

    return pitch, roll

def get_bias(sensor, num_samples):
    pitch_bias_total = 0
    roll_bias_total = 0

    for _ in range(num_samples):
        accel_data, gyro_data = read_sensor_data(sensor)
        pitch, roll = complementary_filter(accel_data, gyro_data, accel_bias, gyro_bias, dt=0.01)
        pitch_bias_total += pitch
        roll_bias_total += roll
        time.sleep(0.01)

    pitch_bias = pitch_bias_total / num_samples
    roll_bias = roll_bias_total / num_samples

    return pitch_bias, roll_bias

# Main function
def main():
    sensor = mpu6050(0x68)
    
    # Define number of samples for bias estimation
    num_samples = 1000

    # Accumulators for biases
    pitch_bias_total = 0
    roll_bias_total = 0

    # Get biases over 100 readings
    for _ in range(num_samples):
        accel_data, gyro_data = read_sensor_data(sensor)
        pitch, roll = complementary_filter(accel_data, gyro_data, accel_bias, gyro_bias, dt=0.01)
        pitch_bias_total += pitch
        roll_bias_total += roll
        time.sleep(0.01)

    # Calculate average biases
    pitch_bias_avg = pitch_bias_total / num_samples
    roll_bias_avg = roll_bias_total / num_samples

    print("Pitch Bias:", math.degrees(pitch_bias_avg))
    print("Roll Bias:", math.degrees(roll_bias_avg))

if __name__ == "__main__":
    main()