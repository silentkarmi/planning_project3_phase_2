#!/usr/bin/env python3
# Author @ Kartikeya Mishra @ Chang-Hong Chen

import cv2
import heapq
from dataclasses import dataclass

from constants import CONSTANT
from canvas import Canvas
from node import Node
from utility import Utility
from obstacles.circleObstacle import CircleObstacle
from obstacles.hexagonObstacle import HexagonObstacle
from obstacles.polygonObstacle import PolygonObstacle

import numpy as np

MAP_RESOLUTION_SCALE = 10
MAP_THRESOLD_REGION = int(0.5 * MAP_RESOLUTION_SCALE)
ANGLE_RESOLUTION = 30 # degree

@dataclass
class Traversal:

    def __init__(self):
        self._closedList = set() 
        self._closeListNodes = []
        self._openList = []
        self._listSolution = []
        
        self.canvaArea = Canvas()
        
        objCircle = CircleObstacle((300, 185), 40)
        self.canvaArea.addObstacle(objCircle)
        
        objTriangularPolygon = PolygonObstacle([(36, 185),
                                                (115, 210),
                                                (80, 180),
                                                (105, 100)])
        self.canvaArea.addObstacle(objTriangularPolygon)
        
        objHexagonPolygon = HexagonObstacle([(165, round(79.79275)),
                                            (165, round(120.2073)),
                                            (200, round(140.4145)),
                                            (235, round(120.2073)),
                                            (235, round(79.79275)),
                                            (200, round(59.5855))])
        self.canvaArea.addObstacle(objHexagonPolygon)
        self.canvaArea.formObstaclesMap()

        # Used for recording closed node in a higher resolution map
        self.closedNodeMap = np.zeros((CONSTANT.CANVAS_HEIGHT * MAP_RESOLUTION_SCALE, 
                                       CONSTANT.CANVAS_WIDTH * MAP_RESOLUTION_SCALE, 
                                       360 // ANGLE_RESOLUTION), np.uint8)
        
        self.startNode = None
        self.endNode = None
        self.solutionNode = None
        
    def expandNodesAndPushIntoWorkspace(self, node):
        listNodes = node.createNodes()
        for subNode in listNodes:
            self.pushNode(subNode)       

    def isNodeClosed(self, node): 
        # Transform x, y cart coord to w, h image coord
        w, h = Utility.getCoordinatesInWorldFrame(node.coord)
        return self.closedNodeMap[h * MAP_RESOLUTION_SCALE, 
                                  w * MAP_RESOLUTION_SCALE, 
                                  Utility.actionInDegree(node.coord[2]) // ANGLE_RESOLUTION] != 0

    def pushNode(self, node):
        if node != None:
            isNodeSafe = self.canvaArea.isOutsideObstacleSpaceByMap(node.coord)
            
            if isNodeSafe:
                if not self.isNodeClosed(node):
                    heapq.heappush(self._openList, node)

                
    def isThisGoalNode(self, nodeToCheck):
        xcentre, ycentre, end_theta = self.endNode.coord
        x, y, node_theta = nodeToCheck.coord
        in_goal = (x - xcentre)**2 + (y -ycentre)**2 - (CONSTANT.GOAL_THRESOLD)**2 < 0
        is_goal = False
        if in_goal:
            if (Utility.actionInDegree(end_theta) == 
                            Utility.actionInDegree(node_theta)):
                is_goal = True

        return is_goal

    def AddtoClosedNodeMap(self, node):
        # Transform x, y cart coord to w, h image coord
        w, h = Utility.getCoordinatesInWorldFrame(node.coord)
        matrix_x = int(h * MAP_RESOLUTION_SCALE - MAP_THRESOLD_REGION)
        matrix_y = int(w * MAP_RESOLUTION_SCALE - MAP_THRESOLD_REGION)
        matrix_degree = Utility.actionInDegree(node.coord[2]) // ANGLE_RESOLUTION
        self.closedNodeMap[matrix_x, matrix_y, matrix_degree] = 1
        
        counter = 0
        for counter_x in range(1, 11):
            for counter_y in range(1, 11):
                self.closedNodeMap[matrix_x + counter_x , 
                                   matrix_y + counter_y, matrix_degree] = 1
 
    
    def createNodeTree(self):
        print("Generating Node Tree...")
        self.pushNode(self.startNode)
        while(self._openList):

            # Run spinning cursor while creating node tree
            Utility.run_spinning_cursor()
            
            # pops an element from the top of the list
            tempNode = heapq.heappop(self._openList)     

            if self.isNodeClosed(tempNode):
                continue

            self._closeListNodes.append(tempNode)
            self._closedList.add((round(tempNode.coord[0]),
                                        round(tempNode.coord[1])))  

            # Add closed node to a higher resolution map
            self.AddtoClosedNodeMap(tempNode)

             
            if(self.isThisGoalNode(tempNode)):
                # print("Total cost: ", tempNode.cost2come)
                self.solutionNode = tempNode
                break
            
            self.expandNodesAndPushIntoWorkspace(tempNode)
            
        else:
            print("SOLUTION NOT FOUND")
            
    def drawNodeTree(self):
        print("Drawing Node Tree...")
        counter = 0
        for tempNode in self._closeListNodes:
            self.canvaArea.drawNode(tempNode)
            counter +=1
            cv2.waitKey(1)
            
    def backTrack(self):
        print("Backtracking...")
        self._listSolution = []
        tempNode = self.solutionNode
        while tempNode != None:
            self._listSolution.append(tempNode)
            tempNode = tempNode.parentNode
            
    def drawSolution(self):  
        print("Drawing the solution...")     
        
        mobileRobotStepSize = CONSTANT.MOBILE_ROBOT_RADIUS - CONSTANT.VECTOR_LEN + CONSTANT.WALL_CLEARANCE
        if mobileRobotStepSize <= 0:
            mobileRobotStepSize = 1
        
        self.canvaArea.drawMobileRobot(self.startNode, CONSTANT.COLOR_YELLOW)
        for node in self._listSolution[::mobileRobotStepSize]:
            self.canvaArea.drawMobileRobot(node)
        self.canvaArea.drawMobileRobot(self.solutionNode, CONSTANT.COLOR_GREEN)
            
        for node in self._listSolution[::-1]:
            self.canvaArea.drawNode(node, CONSTANT.COLOR_BLUE)
            
        cv2.waitKey(1)
