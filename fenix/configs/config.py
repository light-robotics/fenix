obstacle = {
    "danger_offset": 1.0,
    "outer_danger_offset": 2.0,
}

leg = {
    "a": 12.7,  # PointA to PointB
    "b": 6.9,   # PointB to PointC
    #"c": 17.2, # PointC to PointD / point toe length
    "c": 19.4,  # PointC to PointD / crescent toe length
    "d": 6.7,   # PointO to PointA
    "mount_point_offset": 3.8,
    "phi_angle" : 2.7 # angle fix due to leg not being straight
}

angles = {
    "to_surface": {
        "min" : -45,
        "max" : 45,
        "step": 1
    },
    "alpha": {
        "min": -50,
        "max": 80
    },
    "beta": {
        "min": -115,
        "max": -20
    },
    "gamma": {
        "min": -110,
        "max": 0
    }
}

fenix = {
    "margin": {
        1: 6,
        2: 6
    },
    "leg_up": {
        1: 6,
        2: 6
    }    
}

mode = 90
