from configs.modes import STAR_LEGS

obstacle = {
    "danger_offset": 1.0,
    "outer_danger_offset": 2.0,
}

leg = {
    "a": 7.4,    # PointA to PointB - femur
    "b": 6.9,    # PointB to PointC - femur-tibia
    "c": 14.7,   # PointC to PointD - tibia / point toe length
    "d": 6.9,    # PointO to PointA - trochanter-coxa
    "mount_point_offset": 3.8, # ???
    "phi_angle" : -22.5 # angle fix due to leg not being straight
}

start = {
    "vertical"                 : 11,
    "horizontal_x"             : 17, # 18
    "horizontal_y"             : 17, # 18
    "y_offset_body"            : 3,
    "initial_z_position_delta" : 5, # 3
}

if not STAR_LEGS:
    modes = {
        "run_mode" : {
            "horizontal_xy" : 16,
        },
        "sentry_mode" : {
            "horizontal_xy" : 16,
        },
        "walking_mode" : {
            "horizontal_xy" : 17,
        },
        "battle_mode" : {
            "horizontal_xy" : 17,
        }
    }

    speed = {
        "run" : 350,
        "hit" : 500,
    }

    moves = {
        "up_or_down_cm"         : 2,
        "move_body_cm"          : 7,
        "forward_body_1_leg_cm" : 8,
        "forward_body_2_leg_cm" : 4,    
        "reposition_cm"         : 1,
        "side_look_angle"       : 12,
        "vertical_look_angle"   : 30,
    }

    fenix = {
        "margin": {
            1: 5,
            2: 6
        },
        "leg_up": {
            1: 7,
            2: 4
        },
        # parameters for moving further, when moving with feedback
        "servos": {
            "diff_from_target_limit": 3.5,
            "diff_from_prev_limit": 0.5
        },
        # issue next command a little faster, than previous is finished executing
        # when moving without feedback
        "movement_command_advance_ms" : 0.05,
        "movement_overshoot_coefficient" : 0.2,
    }

else:
    modes = {
        "run_mode" : {
            "horizontal_xy" : 16,
        },
        "sentry_mode" : {
            "horizontal_xy" : 16,
        },
        "walking_mode" : {
            "horizontal_xy" : 17,
        },
        "battle_mode" : {
            "horizontal_xy" : 17,
        }
    }

    speed = {
        "run" : 250,
        "hit" : 500,
    }

    moves = {
        "up_or_down_cm"         : 2,
        "move_body_cm"          : 7,
        "forward_body_1_leg_cm" : 8,
        "forward_body_2_leg_cm" : 6,    
        "reposition_cm"         : 1,
        "side_look_angle"       : 12,
        "vertical_look_angle"   : 45,
    }

    fenix = {
        "margin": {
            1: 3,
            2: 6
        },
        "leg_up": {
            1: 6,
            2: 5
        },
        # parameters for moving further, when moving with feedback
        "servos": {
            "diff_from_target_limit": 3.5,
            "diff_from_prev_limit": 0.5
        },
        # issue next command a little faster, than previous is finished executing
        # when moving without feedback
        "movement_command_advance_ms" : 0.05,
        "movement_overshoot_coefficient" : 0.2,
    }




limits = {
    "body_forward"        : 7,
    "body_backward"       : 7,
    "body_sideways"       : 7,
    "side_look_angle"     : 24,
    "vertical_look_angle" : 30,
}

angles = {
    "to_surface": {
        "min" : -90,
        "max" : 90,
        "step": 1,
        "ideal": 20
    },
    "alpha": {
        "min": -30,
        "max": 55
    },
    "beta": {
        "min": -120,
        "max": -10
    },
    "gamma": {
        "min": -100 + leg["phi_angle"],
        "max": -15 + leg["phi_angle"]
    },
    "front_leg": {
        "alpha": {
            "min": -30,
            "max": 78
        },
        "beta": {
            "min": -115,
            "max": 0
        },
        "gamma": {
            "min": -100 + leg["phi_angle"],
            "max": -12 + leg["phi_angle"]
        },
        "tetta": {
            "min": -27,
            "max": 27
        },
        "beta-gamma": {
            "min": 20 + leg["phi_angle"],
            "max": 170 + leg["phi_angle"]
        }
    },
    "rear_leg": {
        "alpha": {
            "min": -30,
            "max": 60
        },
        "beta": {
            "min": -105,
            "max": 0
        },
        "gamma": {
            "min": -100 + leg["phi_angle"],
            "max": -12 + leg["phi_angle"]
        },
        "tetta": {
            "min": -20,
            "max": 20
        },
        "beta-gamma": {
            "min": 20 + leg["phi_angle"],
            "max": 170 + leg["phi_angle"]
        }
    }
}

mode = 90
