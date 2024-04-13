import json

from rplidar import RPLidar


class FenixLidar(RPLidar):
    def __init__(self, port='/dev/ttyUSB0'):
        super().__init__(port=port)
        self.motor_speed = 0
        self.current_height = 0
        self.measurements = []

    def scan_front(self, iterations=1):
        print('Scanning started')
        self.connect()
        self.reset()
        self.clean_input()
        with open('/fenix/fenix/wrk/gyroaccel_data.txt', 'r') as f:
            pitch, roll = f.readline().split(',')
        for i, scan in enumerate(self.iter_scans()):            
            for value in scan:
                # (quality, angle, distance)
                if 15 < value[1] < 165:
                    self.measurements.append({
                        "height": self.current_height,
                        "angle": value[1],
                        "distance": value[2],
                        "pitch": float(pitch),
                        "roll": float(roll),
                    })
            
            if i > iterations:
                break
        #self.stop()
        #self.stop_motor()
        self.disconnect()
        #self.motor_speed = 0
        
        print('Scanning finished')
        self.save_data()
    
    def save_data(self):
        print('Saving data')
        with open('/fenix/fenix/logs/lidar_data.log', 'a') as f:
            for item in self.measurements:
                f.write(json.dumps(item)+'\n')
        print('Data saved')

if __name__ == '__main__':
    import time
    
    for j in range(2):
        lidar = FenixLidar('/dev/ttyUSB0')
        lidar.scan_front()
        #del lidar
        time.sleep(3)
        
    #lidar.save_data()
