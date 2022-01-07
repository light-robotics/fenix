from typing import Optional, Dict
from configs import config as cfg
import configs.code_config as code_config
import logging.config


class ConstraintsChecker:
    def __init__(self):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')

    @staticmethod
    def rule_followed(constraint: Dict, value: float) -> bool:
        if value > constraint["max"] or value < constraint["min"]:
            return False
        return True

    def leg_angles_correct(
        self,
        leg_type: str,
        alpha: Optional[float] = None, 
        beta: Optional[float] = None, 
        gamma: Optional[float] = None, 
        tetta: Optional[float] = None
        ) -> bool:
        
        if tetta is None and alpha is None and beta is None and gamma is None:
            self.logger.info('All angles provided are None')
            raise Exception('All angles provided are None')
        
        leg_constraints = cfg.angles[leg_type]
        if tetta is not None:
            if not self.rule_followed(leg_constraints["tetta"], tetta):
                self.logger.info(f'Bad tetta : {tetta}')
                return False
        
        if alpha is not None:
            if not self.rule_followed(leg_constraints["alpha"], alpha):
                self.logger.info(f'Bad alpha : {alpha}')
                return False
        
            if not self.rule_followed(leg_constraints["beta"], beta):
                self.logger.info(f'Bad beta : {beta}')
                return False
        
            if not self.rule_followed(leg_constraints["gamma"], gamma):
                self.logger.info(f'Bad gamma : {gamma}')
                return False
            
            if not self.rule_followed(leg_constraints["beta+gamma"], beta+gamma):
                self.logger.info(f'Bad beta+gamma : {beta+gamma}')
                return False

        self.logger.info(f'Good angles : {alpha}, {beta}, {gamma}, {tetta}')
        return True