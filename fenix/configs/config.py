obstacle = {
    "danger_offset": 2.0,
    "outer_danger_offset": 3.0,
}

leg = {
    "a": 15,    # PointA to PointB - femur
    "b": 6.5,
    "c": 15.5, # 18,    # PointB to PointC - femur-tibia
    "d": 6.5,    # PointO to PointA - trochanter-coxa
    "mount_point_offset": 3.8, # ???
    #"phi_angle" : -22.5 # angle fix due to leg not being straight
}

start = {
    "vertical"                 : 12,
    "horizontal_x"             : 18, # 15
    "horizontal_y"             : 18, # 15
    "x_offset_body"            : 0,
    "y_offset_body"            : 0,
    "initial_z_position_delta" : 5, # 3
}

modes = {
    "run_mode" : {
        "x": 17,
        "y": 17,
    },
    "sentry_mode" : {
        "x": 15,
        "y": 15,
    },
    "walking_mode" : {
        "x": 18,
        "y": 18,
    },
    "battle_mode" : {
        "x": 15,
        "y": 15,
    }
}

speed = {
    "run" : 350,
    "hit" : 500,
}

moves = {
    "up_or_down_cm"         : 4,
    "move_body_cm"          : 7,
    "forward_body_1_leg_cm" : 8,
    "forward_body_2_leg_cm" : 7,    
    "reposition_cm"         : 1,
    "side_look_angle"       : 12,
    "vertical_look_angle"   : 6,
}

fenix = {
    "margin": {
        1: 5,
        2: 6
    },
    "leg_up": {
        1: 10,
        2: 7
    },
    # parameters for moving further, when moving with feedback
    "servos": {
        "diff_from_target_limit": 3.5,
        "diff_from_prev_limit": 0.5
    },
    # issue next command a little faster, than previous is finished executing
    # when moving without feedback
    "movement_command_advance_ms" : -0.05,
    "movement_overshoot_coefficient" : 0.0,
    "balance_offset": 2.5,
}

limits = {
    "body_forward"        : 7,
    "body_backward"       : 7,
    "body_sideways"       : 7,
    "side_look_angle"     : 24,
    "vertical_look_angle" : 30,
}

"""
"tetta": 80,
"tetta_alpha_down": {
    "tetta": 90,
    "alpha": 30,
},
"tetta_alpha_up": {
    "tetta": 45,
    "alpha": 85,
},
"""
angles_limits = {
    
    "alpha": {
        "min": -70,
        "max": 90
    },
    "beta": {
        "min": -140,
        "max": 75,
    },
    "gamma": {
        "min": -110,
        "max": 110,
    }
}
