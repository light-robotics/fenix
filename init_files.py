from pathlib import Path
import os

Path("/fenix/fenix/logs/").mkdir(parents=True, exist_ok=True)

with open('/boot/firmware/config.txt', 'r') as f:
    contents = f.readlines()

arducam_pivariety_found = False
uart3_found = False
uart4_found = False
uart5_found = False
uart0_found = False

for item in contents:
    if item.strip() == 'dtoverlay=arducam-pivariety':
        arducam_pivariety_found = True
    if item.strip() == 'dtoverlay=uart3':
        uart3_found = True
    if item.strip() == 'dtoverlay=uart4':
        uart4_found = True
    if item.strip() == 'dtoverlay=uart5':
        uart5_found = True
    if item.strip() == 'dtoverlay=uart0':
        uart0_found = True

if not arducam_pivariety_found:
    print("dtoverlay=arducam-pivariety not found in config. Adding")
    os.system('sudo bash -c "echo dtoverlay=arducam-pivariety >> /boot/firmware/config.txt"')
if not uart3_found:
    print("dtoverlay=uart3 not found in config. Adding")
    os.system('sudo bash -c "echo dtoverlay=uart3 >> /boot/firmware/config.txt"')
if not uart4_found:
    print("dtoverlay=uart4 not found in config. Adding")
    os.system('sudo bash -c "echo dtoverlay=uart4 >> /boot/firmware/config.txt"')
if not uart5_found:
    print("dtoverlay=uart5 not found in config. Adding")
    os.system('sudo bash -c "echo dtoverlay=uart5 >> /boot/firmware/config.txt"')
if not uart0_found:
    print("dtoverlay=uart0 not found in config. Adding")
    os.system('sudo bash -c "echo dtoverlay=uart0 >> /boot/firmware/config.txt"')

# sudo raspi-config
# enable serial port
