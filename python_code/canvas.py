import numpy as np
import cv2
from dataclasses import dataclass

from constants import CONSTANT
from node import Node
from utility import Utility


@dataclass
class Canvas:
    def __init__(self):
        # Create a black image
        self._canvasArea = np.zeros((CONSTANT.CANVAS_HEIGHT, CONSTANT.CANVAS_WIDTH, 3), np.uint8)
        self._obstacles = []
        self._obstaclesMap = np.zeros((CONSTANT.CANVAS_HEIGHT, CONSTANT.CANVAS_WIDTH), np.uint8)
        
    def addObstacle(self, objObstacle):
        self._obstacles.append(objObstacle)

    def drawObstacles(self):
        for objObstacle in self._obstacles:
            objObstacle.draw(self._canvasArea)
        cv2.imshow(CONSTANT.WINDOW_NAME, self._canvasArea)
        cv2.waitKey(1)
        
    def drawMobileRobot(self, node, color = CONSTANT.COLOR_LIGHT_BLUE):
        if isinstance(node, Node):
            cv2.circle(self._canvasArea , 
                     Utility.getCoordinatesInWorldFrame(node.coord),
                     CONSTANT.MOBILE_ROBOT_RADIUS, 
                     color, -1)
            cv2.imshow(CONSTANT.WINDOW_NAME, self._canvasArea)
            
    def drawNode(self, node, color = CONSTANT.COLOR_WHITE):
        if isinstance(node, Node) and node.parentNode is not None:
            cv2.line(self._canvasArea , 
                     Utility.getCoordinatesInWorldFrame(node.parentNode.coord), 
                     Utility.getCoordinatesInWorldFrame(node.coord), 
                     color)
            cv2.imshow(CONSTANT.WINDOW_NAME, self._canvasArea)
            
    def isOutsideObstacleSpace(self, coord):
        isValid = True
        
        isValid = Node.isCoordValid(coord)
        if isValid:
            for objObstacle in self._obstacles:
                isValid = objObstacle.isOutside(coord)
                if isValid == False:
                    break
            
        return isValid

    def formObstaclesMap(self): 
        for h in range(0, CONSTANT.CANVAS_HEIGHT): 
            for w in range(0, CONSTANT.CANVAS_WIDTH): 
                x, y = Utility.getCoordinatesInWorldFrame((w, h))
                if not self.isOutsideObstacleSpace((x, y, 0)):
                    self._obstaclesMap[h, w] = 255
        return self._obstaclesMap

    def isOutsideObstacleSpaceByMap(self, coord):
        w, h = Utility.getCoordinatesInWorldFrame(coord)

        return self._obstaclesMap[h, w] == 0 
    
