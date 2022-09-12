import os


DEBUG = False # in DEBUG mode command to servos is not issued

project_dir = os.path.join(os.path.dirname(__file__), '..')
main_log_file = os.path.join(project_dir, 'logs', 'main.log')
current_sensor_log_file = os.path.join(project_dir, 'logs', 'current_sensor.log')
pathfinding_log_file = os.path.join(project_dir, 'logs', 'pathfinding.log')
angles_log_file = os.path.join(project_dir, 'logs', 'angles.log')

movement_command_file = os.path.join(project_dir, 'wrk', 'movement_command.txt')
neopixel_command_file = os.path.join(project_dir, 'wrk', 'neopixel_command.txt')

cache_dir = os.path.join(project_dir, 'cache')

logger_config = {
    'version': 1,
    'formatters': {
        'default_formatter': {
            'format': '[%(asctime)s][%(levelname)s] %(message)s'
        },
    },
    'handlers': {
        'main_file_handler': {
            'class': 'logging.FileHandler',
            'formatter': 'default_formatter',
            'filename': main_log_file
        },
        'current_sensor_file_handler': {
            'class': 'logging.FileHandler',
            'formatter': 'default_formatter',
            'filename': current_sensor_log_file
        },
        'pathfinding_file_handler': {
            'class': 'logging.FileHandler',
            'formatter': 'default_formatter',
            'filename': pathfinding_log_file
        },
        'angles_file_handler': {
            'class': 'logging.FileHandler',
            'formatter': 'default_formatter',
            'filename': angles_log_file
        }
    },
    'loggers': {
        'main_logger': {
            'handlers': ['main_file_handler'],
            'level': 'DEBUG',
            'propagate': True
        },
        'current_sensor_logger': {
            'handlers': ['current_sensor_file_handler'],
            'level': 'DEBUG',
            'propagate': True
        },
        'pathfinding_logger': {
            'handlers': ['pathfinding_file_handler'],
            'level': 'DEBUG',
            'propagate': True
        },
        'angles_logger': {
            'handlers': ['angles_file_handler'],
            'level': 'DEBUG',
            'propagate': True
        }
    }
}
