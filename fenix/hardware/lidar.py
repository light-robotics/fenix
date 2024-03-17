import logging
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import logging.config
import configs.code_config as code_config

from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

for i, scan in enumerate(lidar.iter_scans()):
    with open('/fenix/fenix/wrk/lidar_data.txt', 'a') as f:
        f.write(f'{scan}\n')
    print(i, len(scan))
    
    found = False
    for value in scan:
        if value[2] < 300:
            print(value) # (quality, angle, distance)
            found = True
    if not found:
        print('None')
    
    if i > 100:
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()
