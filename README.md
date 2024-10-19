# fenix
Fenix quadruped robot.
YouTube channel: https://www.youtube.com/channel/UC5iMcYcLpUnzzhuc-a_uPiQ
Email: light.robotics.2020@gmail.com

To run fenix:
- sudo /fenix/venv/bin/python fenix/run/neopixel_commands_reader.py
- python fenix/fenix_hardware/fenix_dualshock.py (DualShock 4v2)
- sudo /fenix/venv/bin/python /fenix/fenix/fenix_hardware/fenix_dualsense.py (DualSense)
- python fenix/core/movement_processor.py
- python /fenix/fenix/hardware/mpu6050_avg.py
- python /fenix/fenix/fenix_hardware/fenix_enders.py
- to stream video:
sudo ffmpeg -s 1024x576 -f video4linux2 -i /dev/video1 -f mpegts -codec:v mpeg1video -b:v 4000k -r 30 http://51.250.26.33:8081/12345/1024/576/

Useful commands:
- sudo chmod 666 /dev/ttyAMA0
