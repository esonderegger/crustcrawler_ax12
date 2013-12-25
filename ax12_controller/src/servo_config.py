import math

servo_param = {
    # shoulder_pan
    1: { 
        'max_speed': 1.5
       },
    # shoulder_tilt A
    2: { 
        'max_speed': 2.0
       },
    # shoulder_tilt B
    3: { 
        'max_speed': 2.0,
        'flipped': True,
       },
    # elbow_flex A
    4: { 
        'max_speed': 2.0,
       },
    # elbow_flex B
    5: { 
        'max_speed': 2.0,
        'flipped': True,
       },
    # wrist_roll
    6: { 
        'max_speed': 2.0
       },
    # gripper
    7: { 
        'max_speed': 2.0,
        'home_encoder': 600,
        'min_ang': -0.15,
        'max_ang': 0.55
       }
}
