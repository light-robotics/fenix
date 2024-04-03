from typing import Optional, Dict
from configs import config as cfg
import configs.code_config as code_config
import logging.config


def rule_followed(constraint: Dict, value: float) -> bool:
    if value > constraint["max"] or value < constraint["min"]:
        return False
    return True

def tettas_correct(tettas: list[float], logger = None) -> bool:
    for idx, value in enumerate(tettas):
        if idx + 1 > len(tettas) - 1:
            plus_index = 0
            plus_value = tettas[0]
        else:
            plus_index = idx + 1
            plus_value = tettas[idx + 1]
        
        if value - plus_value > 80:
            logger.info(f'Bad value: {value - plus_value} for {(idx + 1, plus_index + 1)}')
            return False

    return True

def leg_angles_correct(
    alpha: Optional[float] = None, 
    beta: Optional[float] = None,
    tetta: Optional[float] = None,
    logger = None
    ) -> bool:
    
    #logger.info(f'Trying angles {[alpha, beta, tetta]}')

    if alpha is None and beta is None and tetta is None:
        logger.info('All angles provided are None')
        raise Exception('All angles provided are None')
    
    #leg_constraints = cfg.angles
    
    if tetta is not None:
        #if not rule_followed(leg_constraints["tetta"], tetta):
        if alpha > 90:
            if abs(tetta) > 50:
                logger.info(f'Bad alpha + tetta up : {alpha}, {tetta}')
                return False
        elif alpha > 30:
            if abs(tetta) > 70:
                logger.info(f'Bad alpha + tetta normal : {alpha}, {tetta}')
                return False
        else:
            if abs(tetta) > 90:
                logger.info(f'Bad alpha + tetta down : {alpha}, {tetta}')
                return False
        
        """
        if abs(tetta) > cfg.angles_limits["tetta_alpha_down"]["tetta"] and \
            alpha > cfg.angles_limits["tetta_alpha_down"]["alpha"]:
            logger.info(f'Bad alpha + tetta down : {alpha}, {tetta}')
            return False
        
        if abs(tetta) > cfg.angles_limits["tetta"]:
            logger.info(f'Bad tetta : {tetta}')
            return False
        
        if abs(tetta) > cfg.angles_limits["tetta_alpha_up"]["tetta"] and \
            alpha > cfg.angles_limits["tetta_alpha_up"]["alpha"]:
            logger.info(f'Bad alpha + tetta up : {alpha}, {tetta}')
            return False
        """
    if alpha is not None:
        #if not rule_followed(leg_constraints["alpha"], alpha):
        if alpha > cfg.angles_limits["alpha"]["max"] or \
            alpha < cfg.angles_limits["alpha"]["min"]:
            logger.info(f'Bad alpha : {alpha}')
            return False
    
        #if not rule_followed(leg_constraints["beta"], beta):
        if beta > cfg.angles_limits["beta"]["max"] or \
            beta < cfg.angles_limits["beta"]["min"]:
            logger.info(f'Bad beta : {beta}')
            return False
    
    #logger.info(f'Good angles : {alpha}, {beta}')
    return True