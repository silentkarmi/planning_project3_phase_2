#!/usr/bin/env python3
# Author @ Kartikeya Mishra

from dataclasses import dataclass

@dataclass
class CONSTANT:
    
    START_NODE = (0, 0, 0)
    GOAL_NODE = (0, 0, 0)
    
    
    SCALE_FACTOR = 0.1 #in cm
    
    CANVAS_WIDTH = 1000 # in cm
    CANVAS_HEIGHT = 1000 # in cm
    
    
    
    # Robot Burger Specs
    MOBILE_ROBOT_RADIUS = 178 * 0.5 * SCALE_FACTOR 
    ROBOT_RADIUS_INNER = 160 * 0.5 * SCALE_FACTOR 
    # WHEEL_DIA = 66 * 0.001 # in metres
    MAX_SPEED_WHEEL = 100 * SCALE_FACTOR # in cm/s
    
    GOAL_THRESOLD = 15 * SCALE_FACTOR   
    
    WALL_CLEARANCE = 200 * SCALE_FACTOR 
    CLEARANCE = WALL_CLEARANCE + MOBILE_ROBOT_RADIUS 
    
    WINDOW_NAME = "Project3_Phase_2"
    
    ACTION_COUNT_FOR_ONE_WHEEL = 3 # 3 + 1, Total = 16 ACTIONS
    TIME_INTERVAL = 1 # in secs
    
   

        
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
    #    ACTIONS = []
    #    SMALLEST_SPEED_UNIT = (CONSTANT.MAX_SPEED_WHEEL) * 2 / CONSTANT.ACTION_COUNT_FOR_ONE_WHEEL
    #    for i in range(CONSTANT.ACTION_COUNT_FOR_ONE_WHEEL + 1):
    #       for j in range(CONSTANT.ACTION_COUNT_FOR_ONE_WHEEL + 1):
    #           ACTIONS.append((CONSTANT.MAX_SPEED_WHEEL - i * SMALLEST_SPEED_UNIT, 
    #                          CONSTANT.MAX_SPEED_WHEEL - j * SMALLEST_SPEED_UNIT))
       
       RPM1 = 22
       RPM2 = 10
       ACTIONS = []
       ACTIONS.append((RPM1, RPM1))
       ACTIONS.append((RPM2, RPM2))
       ACTIONS.append((-RPM1, RPM1))
       ACTIONS.append((RPM1, -RPM1))
       ACTIONS.append((RPM1, 0))
       ACTIONS.append((0, RPM1))
       ACTIONS.append((RPM1, RPM2))
       ACTIONS.append((RPM2, RPM1))
    #    ACTIONS.append((RPM2, RPM2))
    #    ACTIONS.append((RPM2, RPM1))
    #    ACTIONS.append((RPM1, RPM2))
    #    ACTIONS.append((RPM1, 0))
    #    ACTIONS.append((0, RPM1))
    #    ACTIONS.append((RPM2, 0))
    #    ACTIONS.append((0, RPM2))
        
       return ACTIONS
