import copy
from typing import List, Dict


class Move:
    def __init__(self, 
                 command: str, # change command to enum some day
                 value: int = 0, 
                 legs: List[int] = [1, 2, 3, 4], 
                 target_legs_position: Dict[int, List[float]] = None
        ):
        self.command = command
        self.value = value
        self.legs = legs[:]
        self.target_legs_position = copy.deepcopy(target_legs_position)

    def __repr__(self):
        return f'Move[{self.command[0]}.{self.value}]'

class Attempt:
    def __init__(self, moves: List[Move], result: int):
        self.moves = moves[:]
        self.result = result
    
    def __repr__(self):
        return f'Result : {self.result}|' + '|'.join([str(x) for x in self.moves])
