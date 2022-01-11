import logging.config
import sys
import os
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.fnx_ina219 import FNX_INA219
import configs.code_config as code_config


if __name__ == '__main__':
    logging.config.dictConfig(code_config.logger_config)
    logger = logging.getLogger('current_sensor_logger')
    ina = FNX_INA219()
    while True:
        try:
            logger.info(ina.read())
            time.sleep(0.02)
        except KeyboardInterrupt:
            break
