from ast import Constant
import math
import numpy as np
import cv2
from dataclasses import dataclass

from . constants import CONSTANT
from . node import Node
from . utility import Utility


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
        
    def drawMobileRobot(self, node, color = CONSTANT.COLOR_LIGHT_BLUE):
        if isinstance(node, Node):
            cv2.circle(self._canvasArea , 
                     Utility.getCoordinatesInWorldFrame(node.coord),
                     int(CONSTANT.MOBILE_ROBOT_RADIUS), 
                     color, -1)
            cv2.imshow(CONSTANT.WINDOW_NAME, self._canvasArea)
            
    def drawNode(self, node, color = CONSTANT.COLOR_WHITE):
        if isinstance(node, Node) and node.parentNode is not None:
            
            # print(node.parentNode)
            # print(node)
            
            x = node.parentNode.coord[0]
            y = node.parentNode.coord[1]
            start_thetha = node.parentNode.coord[2]
            
            end_thetha = node.coord[2]
            r = node.coord[3]
            
            action = node.coord[4]
            
            # start_thetha = min(int(start_thetha), int(end_thetha))
            # end_thetha = max(int(start_thetha), int(end_thetha))
            
            if action[0] == action[1]:
                cv2.line(self._canvasArea , 
                     Utility.getCoordinatesInWorldFrame(node.parentNode.coord), 
                     Utility.getCoordinatesInWorldFrame(node.coord), 
                     color)
            else:
                    # curveXpt = []
                    # curveYpt = []
                pts = []
                start = 0
                end = 0
                if start_thetha < end_thetha:
                    start = int(start_thetha)
                    end = int(end_thetha)
                    for i in range(start, end + 1, 1):
                        curveXpt = (x + r * math.cos(math.radians(i)))
                        curveYpt = (y + r * math.sin(math.radians(i)))
                        pts.append(Utility.getCoordinatesInWorldFrame((curveXpt,curveYpt)))
                else:
                    start = int(start_thetha)
                    end = int(end_thetha)
                    for i in range(start, end - 1, -1):
                        curveXpt = (x + r * math.cos(math.radians(i)))
                        curveYpt = (y + r * math.sin(math.radians(i)))
                        pts.append(Utility.getCoordinatesInWorldFrame((curveXpt,curveYpt)))
                
                # print(start, end)
                
                    
                pts = np.array(pts, np.int32)
                pts = pts.reshape((-1, 1, 2))
                
                vL, vR = action
                if vL < vR:
                    cv2.polylines(self._canvasArea, [pts], False, CONSTANT.COLOR_YELLOW)
                else:
                    cv2.polylines(self._canvasArea, [pts], False, CONSTANT.COLOR_ORANGE)

        else:
            origin_x = node.coord[0]
            origin_y = node.coord[1]
            origin_thetha = node.coord[2]
            origin_parent_x = origin_x - CONSTANT.MOBILE_ROBOT_RADIUS * math.cos(math.radians(origin_thetha))
            origin_parent_y = origin_y - CONSTANT.MOBILE_ROBOT_RADIUS * math.sin(math.radians(origin_thetha))
            parent_thetha = 0
            parent_origin = (origin_parent_x, origin_parent_y, parent_thetha)
            cv2.line(self._canvasArea , 
                     Utility.getCoordinatesInWorldFrame(parent_origin), 
                     Utility.getCoordinatesInWorldFrame(node.coord), 
                     color)
        
        
        cv2.imshow(CONSTANT.WINDOW_NAME, self._canvasArea)
        cv2.waitKey(1)
            
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
    
