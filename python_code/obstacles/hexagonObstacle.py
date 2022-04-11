#!/usr/bin/env python3
# Author @ Kartikeya Mishra

import cv2
import math
import numpy as np
from dataclasses import dataclass
from obstacles.polygonObstacle import PolygonObstacle

from constants import CONSTANT

@dataclass
class HexagonObstacle(PolygonObstacle):
    points: list()
               
    def isOutside(self, coord):
        # created line equations to describe the hexagonal polygon
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
                
            lineEquation = 0
            slope = 0
            c = 0
            
            if x2 - x1 == 0:
                if i == 0:
                    lineEquation = x - (x1 - CONSTANT.CLEARANCE)
                elif i == 3:
                    lineEquation = x - (x1 + CONSTANT.CLEARANCE)
            else:
                slope = (y2 - y1) / (x2 - x1)
                c = y1 - slope * x1
                       
                difference = (CONSTANT.CLEARANCE * math.sqrt(1 + slope**2))
                                
                if i == 1:
                    c = c + difference
                elif i == 2:
                    c = c + difference
                elif i == 4:
                    c = c - difference
                elif i == 5:
                    c = c - difference
                elif i == 6:
                    c = c + difference
            
                lineEquation = y - slope * x - c
                
            linePlaneEquationValues.append(lineEquation)
            
        result = (linePlaneEquationValues[0] < 0 or 
                  linePlaneEquationValues[1] > 0 or
                  linePlaneEquationValues[2] > 0 or
                  linePlaneEquationValues[3] > 0 or
                  linePlaneEquationValues[4] < 0 or
                  linePlaneEquationValues[5] < 0 )
        
        return result