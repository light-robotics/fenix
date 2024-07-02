import math
import time
from mpu6050 import mpu6050

def read_sensor_data(sensor: mpu6050):
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    return accel_data, gyro_data

accel_bias = {'x': -0.8573947391235348, 'y': 0.5004527904174809, 'z': 9.378830105346701}
gyro_bias = {'x': 0.5850839694656488, 'y': -1.7712290076335853, 'z': 1.1650305343511445}

accel_bias = {'x': 0, 'y': 0, 'z': 0}  # Update with your accelerometer bias values
gyro_bias = {'x': 0, 'y': 0, 'z': 0}

def complementary_filter(accel_data, gyro_data, dt, alpha=0.98):
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

    # Yaw estimation using gyroscope data (gyroscopic drift may occur over time)
    yaw = 0  # Placeholder value for yaw estimation without magnetometer

    return pitch, roll, yaw

def average_angles(sensor, num_samples):
    pitch_avg = 0
    roll_avg = 0
    yaw_avg = 0

    for _ in range(num_samples):
        accel_data, gyro_data = read_sensor_data(sensor)
        pitch, roll, yaw = complementary_filter(accel_data, gyro_data, dt=0.01)
        pitch_avg += pitch
        roll_avg += roll
        yaw_avg += yaw
        time.sleep(0.01)

    pitch_avg /= num_samples
    roll_avg /= num_samples
    yaw_avg /= num_samples

    return pitch_avg, roll_avg, yaw_avg


# Main loop
def main():
    sensor = mpu6050(0x68)

    pitch_bias = 2.0
    roll_bias = 2.6

    while True:
        start_time = time.time()

        # Calculate average angles over 5 readings
        pitch_avg, roll_avg, yaw_avg = average_angles(sensor, 20)

        pitch_avg -= math.radians(pitch_bias)
        roll_avg -= math.radians(roll_bias)

        pitch = round(math.degrees(pitch_avg), 2)
        roll = round(math.degrees(roll_avg), 2)
        # Print the results
        print("Pitch (average of 5 readings):", pitch)
        print("Roll (average of 5 readings):", roll, "\n")
        #print("Yaw (average of 5 readings):", math.degrees(yaw_avg))
        with open("/fenix/fenix/wrk/gyroaccel_data.txt", "w") as f:
            f.write(f'{pitch},{roll}')

        # Adjust the filter after each measure
        time.sleep(max(0, 0.2 - (time.time() - start_time)))  # Ensure a 0.1-second interval

def single_scan():
    sensor = mpu6050(0x68)

    pitch_bias = 2.0
    roll_bias = 2.6

    start_time = time.time()

    # Calculate average angles over 5 readings
    pitch_avg, roll_avg, yaw_avg = average_angles(sensor, 20)

    pitch_avg -= math.radians(pitch_bias)
    roll_avg -= math.radians(roll_bias)

    pitch = round(math.degrees(pitch_avg), 2)
    roll = round(math.degrees(roll_avg), 2)
    # Print the results
    print("Pitch (average of 5 readings):", pitch)
    print("Roll (average of 5 readings):", roll, "\n")

    return round(roll)

if __name__ == "__main__":
    main()
    #print(single_scan())
