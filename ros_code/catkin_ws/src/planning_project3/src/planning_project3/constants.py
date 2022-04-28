#!/usr/bin/env python3
# Author @ Kartikeya Mishra

from dataclasses import dataclass

@dataclass
class CONSTANT:
    
    START_NODE = (0, 0, 0)
    GOAL_NODE = (0, 0, 0)

    SCALE = 0.1
    CANVAS_WIDTH = int(3500 * SCALE)
    CANVAS_HEIGHT = int(1000 * SCALE)
    
    
    
    # Robot Burger Specs
    MOBILE_ROBOT_RADIUS = 178 * 0.5 * SCALE 
    ROBOT_RADIUS_INNER = 160 * 0.5  * SCALE 

     
    
    GOAL_THRESOLD = 400  * SCALE 
    
    WALL_CLEARANCE = 60  * SCALE 
    CLEARANCE = WALL_CLEARANCE + MOBILE_ROBOT_RADIUS 
    
    WINDOW_NAME = "Project3_Phase_2"
    
    MAX_SPEED_WHEEL = 30 * SCALE
    ACTION_COUNT_FOR_ONE_WHEEL = 2 # 3 + 1, Total = 16 ACTIONS
    TIME_INTERVAL = 0.5 # in secs
    
   
    # Internal Constants
    
    ORIGIN_POINT_OFFSET = CANVAS_HEIGHT
    COLOR_RED = (0,0,255)
    COLOR_YELLOW = (0, 215, 255)
    COLOR_ORANGE = (255, 215, 0)
    COLOR_GREEN = (0, 100, 0)
    COLOR_BLUE = (255, 0, 0)
    COLOR_LIGHT_BLUE = (230, 216, 117)
    COLOR_WHITE = (255, 255, 255)
    
    @staticmethod
    def get_action_sets():
       ACTIONS = []
    #    SMALLEST_SPEED_UNIT = (CONSTANT.MAX_SPEED_WHEEL) * 2 / CONSTANT.ACTION_COUNT_FOR_ONE_WHEEL
    #    for i in range(CONSTANT.ACTION_COUNT_FOR_ONE_WHEEL + 1):
    #       for j in range(CONSTANT.ACTION_COUNT_FOR_ONE_WHEEL + 1):
    #           ACTIONS.append((CONSTANT.MAX_SPEED_WHEEL - i * SMALLEST_SPEED_UNIT, 
    #                          CONSTANT.MAX_SPEED_WHEEL - j * SMALLEST_SPEED_UNIT))

       RPM1 = 50 * CONSTANT.SCALE 
       RPM2 = 6 * CONSTANT.SCALE 
       ACTIONS = []
       ACTIONS.append((RPM1, RPM1))
       ACTIONS.append((RPM2, RPM2))
       ACTIONS.append((RPM1, 0))
       ACTIONS.append((0, RPM1))
       ACTIONS.append((RPM2, 0))
       ACTIONS.append((0, RPM2))
    #    ACTIONS.append((RPM2, RPM1))
    #    ACTIONS.append((RPM1, RPM2))
       
        
       return ACTIONS
