log_file = '//fnx//logs//log.log'
logger_config = {
    'version': 1,
    'formatters': {
        'default_formatter': {
            'format': '[%(asctime)s][%(levelname)s] %(message)s'
        },
    },
    'handlers': {
        'file_handler': {
            'class': 'logging.FileHandler',
            'formatter': 'default_formatter',
            'filename': log_file
        }
    },
    'loggers': {
        'main_logger': {
            'handlers': ['file_handler'],
            'level': 'DEBUG',
            'propagate': True
        }
    }
}
