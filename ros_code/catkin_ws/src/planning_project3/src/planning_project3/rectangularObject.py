#!/usr/bin/env python3
# Author @ Kartikeya Mishra

import cv2
from dataclasses import dataclass
from obstacleInterface import ObstacleInterface

from constants import CONSTANT
from utility import Utility
import numpy as np
import math

@dataclass
class RectangularObstacle(ObstacleInterface):
    points: list()
    
    def draw(self, canvasArea):
        pointsOpenCVReference = []
        for point in self.points:
            pointsOpenCVReference.append(Utility.getCoordinatesInWorldFrame(point))
        
        npPoints = np.array(pointsOpenCVReference)
        cv2.fillPoly(canvasArea, pts=[npPoints], color = CONSTANT.COLOR_RED)
        
    def isOutside(self, coord):
        
        result = False
        x = coord[0]
        y = coord[1]
        
        x1, y1 = self.points[1]
        x2, y2 = self.points[3]
        
        result = (x > x1 - CONSTANT.CLEARANCE and 
                  x < x2 + CONSTANT.CLEARANCE and 
                  y > y2 - CONSTANT.CLEARANCE and 
                  y < y1 + CONSTANT.CLEARANCE)
        
        return not result
