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
    "vertical"      : 14,
    "horizontal_x"  : 17,
    "horizontal_y"  : 17,
    "y_offset_body" : 3

}

angles = {
    "to_surface": {
        "min" : -45,
        "max" : 45,
        "step": 1,
        "ideal": 70
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
            "max": 80
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

fenix = {
    "margin": {
        1: 3,
        2: 6
    },
    "leg_up": {
        1: 5,
        2: 2
    }    
}

mode = 90

moves = {
    "up_or_down_cm"         : 1,
    "move_body_cm"          : 2,
    "forward_body_1_leg_cm" : 7,
    "forward_body_2_leg_cm" : 4,    
    "reposition_cm"         : 1,
}
