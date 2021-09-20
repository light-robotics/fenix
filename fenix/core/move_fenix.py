import time
import datetime
import copy

from commands import get_command

#from sequences import get_values_for_servos
from cyber_core.get_sequence_refactored import calculate_sequence
from cyber_core.kinematics_refactored import MovementSequence
from servos import Fenix


#inas = FNX_INA219()

ms = MovementSequence(legs_offset_v=-10, legs_offset_h=16)

fnx = Fenix()
#fnx.set_speed(500)
movement_speed = 500
run_up_speed = 250 # 130
run_down_speed = 250 # 130 # 200

start = True
while True:
    command = get_command()
    
    if command == 'exit':
        break

    if command != 'none' and bool(command.strip()):
        try:
            ms_cp = copy.deepcopy(ms)
            sequence = calculate_sequence(ms, command)
        except Exception as e:
            print(f'Could not process command - {str(e)}')
            ms = copy.deepcopy(ms_cp)
            time.sleep(2.0)
            continue

        if sequence is None:
            print('Sequence is None')
            time.sleep(0.5)
        else:
        
            if 'start' in command or 'end' in command:
                fnx.set_speed(800)
            else:
                fnx.set_speed(movement_speed)

            start_time = datetime.datetime.now()
            if 'r2l' in command:
                for index, angles in enumerate(sequence):
                    if index % 2 == 1:
                        fnx.set_speed(run_down_speed)
                    else:
                        fnx.set_speed(run_up_speed)
                    fnx.set_servo_values_paced(angles)
            else:
                for angles in sequence:
                    fnx.set_servo_values_paced(angles)
            print(f'Step took : {datetime.datetime.now() - start_time}')

    else:
        time.sleep(0.25)