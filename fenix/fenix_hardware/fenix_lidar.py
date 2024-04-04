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
        self.clean_input()
        for i, scan in enumerate(self.iter_scans()):            
            for value in scan:
                # (quality, angle, distance)
                if 45 < value[1] < 135:
                    self.measurements.append({
                        "height": self.current_height,
                        "angle": value[1],
                        "distance": value[2],
                    })
            
            if i > iterations:
                break

        #self.motor_speed = 0
        print('Scanning finished')
    
    def save_data(self):
        print('Saving data')
        with open('/fenix/fenix/logs/lidar_data.log', 'a') as f:
            for item in self.measurements:
                f.write(json.dumps(item)+'\n')
        print('Data saved')

if __name__ == '__main__':
    import time
    lidar = FenixLidar('/dev/ttyUSB0')
    for j in range(2):
        try:
            lidar.scan_front()
        except:
            lidar.scan_front()
        time.sleep(3)
    lidar.save_data()
