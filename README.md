# fenix
Fenix quadruped robot.
YouTube channel: https://www.youtube.com/channel/UC5iMcYcLpUnzzhuc-a_uPiQ
Email: light.robotics.2020@gmail.com

To run fenix:
- run sudo /fenix/venv/bin/python fenix/run/neopixel_commands_reader.py
- run python fenix/fenix_hardware/fenix_dualshock.py
- run python fenix/core/movement_processor.py
- python /fenix/fenix/hardware/mpu6050_avg.py
- to stream video:
sudo ffmpeg -s 1024x576 -f video4linux2 -i /dev/video0 -f mpegts -codec:v mpeg1video -b:v 4000k -r 30 http://{nexus_ip}:8081/12345/1024/576/

Useful commands:
- sudo chmod 666 /dev/ttyAMA0
