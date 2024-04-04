from dataclasses import dataclass

@dataclass
class NextStatus:
    """
    Next status and action to achieve it
    """
    status: int | None
    action: str

@dataclass
class MoveMapping:
    start: NextStatus
    next: dict[int, NextStatus]
    exit: dict[int, NextStatus]

class CommandsForwarder:
    moves = {
        'forward_two_legged': MoveMapping(
            start=NextStatus(1, 'forward_1'),
            next = {
                1: NextStatus(2, 'forward_2'),
                2: NextStatus(1, 'forward_3'),
            },
            exit = {
                1: NextStatus(None, 'forward_22'),
                2: NextStatus(None, 'forward_32'),
            }
        ),
        'backward_two_legged': MoveMapping(
            start=NextStatus(1, 'backward_1'),
            next = {
                1: NextStatus(2, 'backward_2'),
                2: NextStatus(1, 'backward_3'),
            },
            exit = {
                1: NextStatus(None, 'backward_22'),
                2: NextStatus(None, 'backward_32'),
            }
        ),
        'strafe_right_two_legged': MoveMapping(
            start=NextStatus(1, 'strafe_right_1'),
            next = {
                1: NextStatus(2, 'strafe_right_2'),
                2: NextStatus(1, 'strafe_right_3'),
            },
            exit = {
                1: NextStatus(None, 'strafe_right_22'),
                2: NextStatus(None, 'strafe_right_32'),
            }
        ),
        'strafe_left_two_legged': MoveMapping(
            start=NextStatus(1, 'strafe_left_1'),
            next = {
                1: NextStatus(2, 'strafe_left_2'),
                2: NextStatus(1, 'strafe_left_3'),
            },
            exit = {
                1: NextStatus(None, 'strafe_left_22'),
                2: NextStatus(None, 'strafe_left_32'),
            }
        ),
    }


    def __init__(self):
        self.current_move = None
        self.current_status = None

    def get_move(self, move):
        print(f'\tCurrent_status: {self.current_status}. Current_move: {self.current_move}')
        if self.current_move:
            if self.current_move == move:
                return self.get_next_move()
            else:
                return self.get_exit_move()
        self.current_move = move
        self.current_status = self.moves[move].start.status
        print('Start')
        print(f'\tCurrent_status: {self.current_status}. Current_move: {self.current_move}')
        return self.moves[move].start.action

    def get_next_move(self):
        next = self.moves[self.current_move].next[self.current_status]
        self.current_status = next.status
        print('Next')
        print(f'\tCurrent_status: {self.current_status}. Current_move: {self.current_move}')
        return next.action
        
    def get_exit_move(self):
        exit_move = self.moves[self.current_move].exit[self.current_status]
        self.current_status = self.current_move = None
        print('Exit')
        print(f'\tCurrent_status: {self.current_status}. Current_move: {self.current_move}')
        return exit_move.action


#cf = CommandsForwarder()

#print(cf.get_move('forward_two_legged'))
#print(cf.get_move('forward_two_legged'))
#print(cf.get_move('forward_two_legged'))
#print(cf.get_move('a'))
#print(cf.get_move('a'))
