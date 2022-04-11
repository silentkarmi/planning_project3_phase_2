#!/usr/bin/env python3
# Author @ Kartikeya Mishra

import cv2
import math
import numpy as np
from dataclasses import dataclass
from obstacles.obstacleInterface import ObstacleInterface

from constants import CONSTANT
from utility import Utility

@dataclass
class PolygonObstacle(ObstacleInterface):
    points: list()
        
    def draw(self, canvasArea):
        pointsOpenCVReference = []
        for point in self.points:
            pointsOpenCVReference.append(Utility.getCoordinatesInWorldFrame(point))
        
        npPoints = np.array(pointsOpenCVReference)
        cv2.fillPoly(canvasArea, pts=[npPoints], color = CONSTANT.COLOR_RED)
        
    def isOutside(self, coord):
        # created line equations to describe the boomerang polygon
        
        def getLineEquation(x1, y1, x2, y2, i):
            value = 0
            slope = 0
            if x2 - x1 == 0:
                value = x1 - x
            else:
                slope = (y2 - y1) / (x2 - x1)
                c = y1 - slope * x1
                
                if(slope != 0):
                    
                    difference = (CONSTANT.CLEARANCE * math.sqrt(1 + slope**2))
                                   
                    if i == 0:
                        c = c + difference
                    elif i == 1:
                        c = c - difference
                    elif i == 2:
                        c = c + difference
                    else:
                        c = c - difference
                
                value = y - slope * x - c
                
                if i == 10:
                    value -= 10
                
            return value
    
        result = True
        linePlaneEquationValues = []
        x, y, _ = coord
        for i in range(len(self.points)):
            x1, y1 = self.points[i]
            
            x2 = -1
            y2 = -1
            if i == len(self.points) - 1:
                x2, y2 = self.points[0]
            else:
                x2, y2 = self.points[i + 1]
                
            linePlaneEquationValues.append(
                getLineEquation(x1, y1, x2, y2, i)
            )
                
            
        linePlaneEquationValues.append(
                getLineEquation(101, 56, 148, 84, 10)
            )
        
        result = (((linePlaneEquationValues[0] > 0 or linePlaneEquationValues[3] < 0 or linePlaneEquationValues[4] < 0) or 
                  (linePlaneEquationValues[1] < 0 and linePlaneEquationValues[2] > 0)))
        
        return result
