main_log_file = '//fnx//logs//log.log'
current_sensor_log_file = '//fnx//logs//current_sensor_log.log'
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
        }
    }
}
