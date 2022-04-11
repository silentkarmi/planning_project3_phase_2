#!/usr/bin/env python3
# Author @ Kartikeya Mishra

import cv2
from dataclasses import dataclass
from obstacles.obstacleInterface import ObstacleInterface

from constants import CONSTANT
from utility import Utility
import numpy as np

@dataclass
class CircleObstacle(ObstacleInterface):
    center: tuple()
    radius: int = 0
    
    def draw(self, canvasArea):
        cv2.circle(canvasArea, Utility.getCoordinatesInWorldFrame(self.center), 
                   self.radius, CONSTANT.COLOR_RED, -1)
        
    def isOutside(self, coord):
        x, y,_ = coord
        xcenter, ycenter = self.center
        result = (x - xcenter) ** 2 + (y - ycenter) ** 2 - (self.radius + CONSTANT.CLEARANCE) ** 2
        return result > 0
