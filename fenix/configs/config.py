obstacle = {
    "danger_offset": 1.0,
    "outer_danger_offset": 2.0,
}

leg = {
    "a": 7.4,    # PointA to PointB - femur
    "b": 6.9,    # PointB to PointC - femur-tibia
    "c": 14.7,   # PointC to PointD - tibia / point toe length 
    #"c": 14.7,  # PointC to PointD / crescent toe length
    "d": 6.9,    # PointO to PointA - trochanter-coxa
    "mount_point_offset": 3.8, # ???
    "phi_angle" : -22.5 # angle fix due to leg not being straight
}

start = {
    "vertical"     : 14,
    "horizontal_x" : 17,
    "horizontal_y" : 17
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
        "min": -110,
        "max": -10
    },
    "gamma": {
        "min": -100,
        "max": -5
    } 
}

fenix = {
    "margin": {
        1: 4,
        2: 6
    },
    "leg_up": {
        1: 5,
        2: 2
    }    
}

mode = 90
